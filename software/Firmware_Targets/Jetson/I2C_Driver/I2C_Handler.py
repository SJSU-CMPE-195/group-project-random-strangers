from __future__ import annotations
import threading
from dataclasses import dataclass, field
from queue import Empty, Queue
from time import time, monotonic
from typing import Callable, ClassVar, Generic, TypeVar
import logging

from smbus2 import SMBus, i2c_msg

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

ResultType = TypeVar("ResultType")
class I2CResult(Generic[ResultType]):
    """Describes the result from an I2C Transaction, functions as a future

    Args:
        transaction: The I2CTransaction that was submitted to the driver
        decoder: An optional function that takes the bytes returned by the
            transaction and decodes it into a more useful type. Leave this
            unset for transactions with no result.
    """
    def __init__(
        self,
        transaction: I2CTransaction,
        decoder: Callable[[bytes], ResultType] | None = None,
    ) -> None:
        self._transaction = transaction
        self._decoder = decoder

    def done(self) -> bool:
        return self._transaction.done.is_set()

    def wait(self, timeout: float | None = None) -> bool:
        return self._transaction.done.wait(timeout)

    def result(self, timeout: float | None = None) -> ResultType | None:
        if not self._transaction.done.wait(timeout):
            raise TimeoutError("I2C transaction did not complete in time")

        if self._transaction.error is not None:
            raise self._transaction.error

        if self._decoder is None:
            return None
        elif self._transaction.readData is None:
            raise RuntimeError("I2C transaction completed without a response")
        else:
            return self._decoder(self._transaction.readData)

@dataclass
class I2CTransaction:
    """Describes one queued I2C write, read, or combined transaction.

    Set ``deviceAddress`` to the 7-bit I2C address.  Set ``registerAddress`` to
    the register address to send before a write/read, or leave it at ``None`` when
    the device does not use a register address. Put outgoing bytes in
    ``writeData`` and set ``readCount`` to the number of bytes to read. For a
    read, the driver appends the bytes received from the bus to that buffer at ''readData''.

    A transaction is submitted by placing it on the driver's queue
    and waiting on ``done``. The worker sets ``done`` after a successful bus
    operation. The worker also uses ``inProgressLock`` while validating and
    executing the transaction, so callers should not modify transaction fields
    concurrently with processing. To reuse a transaction, clear the ''done'' event.
    """

    deviceAddress: int = 0
    registerAddress: int | None = None

    writeData: bytes = bytes(0)

    readCount: int = 0
    readData: bytes | None = field(default=None, init=False)

    inProgressLock: threading.Lock = field(default_factory=threading.Lock, init=False)
    error: Exception | None = field(default=None, init=False)
    done: threading.Event = field(default=threading.Event(), init=False)
    

class Jetson_I2C_Handler:
    """Owns one i2c bus and asynchronously fulfuls transactions added to the Queue

    Only one active instance is allowed per I2C bus. The caller owns the
    transaction queue and is responsible for starting ``runThread`` with
    ``start()`` after construction.  Devices must be registered before their
    addresses can be used by ordinary transactions.
    """

    # One driver instance per I2C bus
    _error_check_period_s: ClassVar[float] = 0.25
    _instances: ClassVar[dict[int, Jetson_I2C_Handler]] = {}
    _instance_lock: ClassVar[threading.Lock] = threading.Lock()

    def __init__(self, i2c_bus_id: int, transaction_queue: Queue[I2CTransaction | None], doErrorChecking: bool = False) -> None:
        """Initialize the singleton driver for ``i2c_bus_id``.

        Args:
            i2c_bus_id: Linux/smbus bus number.
            transaction_queue: Queue consumed by :meth:`_runner_task`.  Put
                ``I2CTransaction`` objects on it; ``None`` is reserved as the
                worker shutdown sentinel.
            doErrorChecking: When true, the worker periodically polls all
                registered devices that advertise error support.  Those polls
                share the bus with normal transactions

        Raises:
            OSError: If ``smbus2`` cannot open the requested bus.
        """

        # Prevent init from resetting an existing instance.
        with self._instance_lock:
            if getattr(self, "_initialized", False):
                return

            try:
                self._i2c_bus_id = i2c_bus_id
                self._transaction_queue: Queue[I2CTransaction | None] = transaction_queue

                self._devices: dict[int, tuple[str, bool]] = {}
                self.errors: dict[str, list[str]] = {}
                self.dataLock: threading.Lock = threading.Lock()

                self._bus = SMBus(i2c_bus_id)

                self.doErrorChecking: bool = doErrorChecking
                self.lastErrorCheck: float = float("NaN")
                self.runThread: threading.Thread = threading.Thread(target=self._runner_task)

                logger.debug(f"Initialized an I2C Driver on BusID:{i2c_bus_id}")
                self._initialized = True

            except OSError:
                #remove failed instances
                self._instances.pop(i2c_bus_id, None)
                raise
    
    def submitTransaction(self, transaction: I2CTransaction) -> None:
        """Submit a transaction to the driver's queue and wait for it to complete.

        Args:
            transaction: Transaction to submit. 
        """

        self._transaction_queue.put(transaction)

    def startDriverThread(self) -> None:
        """Start the worker thread that consumes the transaction queue.
        """

        with self.dataLock:
            if(self.runThread.is_alive()):
                logger.debug(f"Starting I2C Driver Runner Thread on BusID:{self._i2c_bus_id} error checking {"enabled" if self.doErrorChecking else "disabled"}")
                self.runThread.start()
            else:
                logger.warning(f"Attempted to start I2C Driver Runner Thread on BusID:{self._i2c_bus_id} but it is already running")

    def stopDriverThread(self) -> None:
        """Stop the worker thread that consumes the transaction queue.
        """

        logger.debug(f"Stopping I2C Driver Runner Thread on BusID:{self._i2c_bus_id}")

        with self.dataLock:
            if getattr(self, "_initialized", False):
                if(self.runThread.is_alive()):
                    self._transaction_queue.put(None) #tell runner to exit
                    self.runThread.join()
                else:
                    logger.warning(f"Attempted to stop I2C Driver Runner Thread on BusID:{self._i2c_bus_id} but it is not running")

    def close(self) -> None:
        """Close this driver's bus and remove it from the singleton registry.
        """

        logger.debug(f"Removing an I2C Driver on BusID:{self._i2c_bus_id}")

        with self._instance_lock:
            if getattr(self, "_initialized", False):
                self._bus.close()
                self._instances.pop(self._i2c_bus_id, None)

                if(self.runThread.is_alive()):
                    self._transaction_queue.put(None) #tell runner to exit
                    self.runThread.join()

                self._initialized = False

    def register_device(self, device_name: str, i2cAddress: int, errors_supported: bool) -> bool:
            """Register a device address for transaction validation.

            Args:
                device_name: Name used as the key in ``errors`` when error
                    polling reports a device error.
                i2cAddress: 7-bit address.  Only ``0x08`` through ``0x77``
                    inclusive are accepted; reserved addresses are rejected.
                errors_supported: If true, periodic error checking queries the
                    device using the driver's standard error protocol.

            Returns:
                ``True`` when the address was newly registered; ``False`` when
                it is outside the accepted range or is already registered.
            """

            if(i2cAddress > 7 and i2cAddress < 120 and i2cAddress not in self._devices): #do not allow reserved addresses or device conflicts
    
                with self.dataLock:
                    self._devices[i2cAddress] = (device_name, errors_supported)
    
                return True
    
            return False

    def update_errors(self) -> None:
            """Poll registered error-reporting devices and return current errors.
            
            
            This method is called automatically by the driver thread if error 
            checking is enabled. This method performs blocking I2C reads/writes.
            """
    
            all_new_errors: dict[str, list[str]] = dict()
            with self.dataLock:
                devices = self._devices.items() #snapshot


            for device_address, (device_name, errors_supported) in devices:
                if(errors_supported == False):
                    continue

                #querry errors
                error_metadata = self._bus.read_i2c_block_data(device_address, 0x1, 5)
                error_integer = int.from_bytes(error_metadata[1:4], byteorder="big", signed=False)
                device_errors: list[str] = list()

                if(error_metadata[0] != 0): #error(s) detected

                    #loop through each set error bit
                    while(error_integer > 0):
                        lowest_set_bit_pos = (error_integer & -error_integer).bit_length() - 1
                        error_integer &= error_integer - 1

                        #command the device to provide the error corresponding to the set bit
                        self._bus.write_byte_data(device_address, 0x6, lowest_set_bit_pos)

                        #get the error string reads a number of bytes = to the first read value
                        error_data = self._bus.read_block_data(device_address, 0x7)

                        device_errors.append("".join(chr(byte) for byte in error_data))

                #add device errors to the error dictionary
                all_new_errors[device_name] = device_errors

            with self.dataLock: #save new errors
                self.lastErrorCheck = time()
                self.errors = all_new_errors

    def getAllErrors(self) -> tuple[float, dict[str, list[str]]]:
        """Return a snapshot of the most recently reported I2C errors.

        Returns:
            A timestamp of when the last errors were read and a dictionary
            whose keys are registered device names and whose values are 
            error messages.
        """

        with self.dataLock:
            return (self.lastErrorCheck, {device_name: errors.copy() for device_name, errors in self.errors.items()})

    def getDeviceErrors(self, deviceName: str) -> tuple[float, list[str]]:
        """Return a snapshot of the most recently reported I2C errors from a specific device.

        Returns:
            A timestamp of when the last errors were read and a list of error messages.
        """
        with self.dataLock:
            device_errors = self.errors.get(deviceName)
            if(device_errors is None):
                logger.warning(f"")
                device_errors = []
            else:
                device_errors = device_errors.copy()

            return (self.lastErrorCheck, device_errors)
        
    def _runner_task(self) -> None:
        """Consume and execute transactions until a ``None`` sentinel arrives.

        Method should be in dedicated thread. Call :meth:`startDriverThread` to 
        do this automatically
        """
        
        next_error_check = monotonic()

        while(True):
            if self.doErrorChecking:
                if(monotonic() >= next_error_check):
                    self.update_errors()
                    next_error_check = monotonic() + self._error_check_period_s

                timeout = max(0.0, next_error_check - monotonic())
                try:
                    next_transaction = self._transaction_queue.get(block=True, timeout=timeout)
                except Empty: #timeout
                    continue

                if(next_transaction is None): #exit thread signal
                    break 

                with next_transaction.inProgressLock:
                    error_reason = self._validate_transaction(next_transaction)
                    if(error_reason is None):
                        try:
                            self._perform_transaction(next_transaction)
                        except Exception as error:
                            next_transaction.error = error
                    
                        next_transaction.done.set()
                    else:
                        logging.warning(error_reason)


            else:
                next_transaction = self._transaction_queue.get(block=True)
                if(next_transaction is None): #exit thread signal
                    break 
                
                with next_transaction.inProgressLock:
                    error_reason = self._validate_transaction(next_transaction)
                    if(error_reason is None):
                        try:
                            self._perform_transaction(next_transaction)
                        except Exception as error:
                            next_transaction.error = error
                            
                        next_transaction.done.set()
                    else:
                        logging.warning(error_reason)

    def _validate_transaction(self, transaction: I2CTransaction) -> str | None: 
        """
            Validate an i2c transaction

            Returns:
                None: if transaction is valid
                str: containing error reason when transaction is invalid
        """

        reasons: list[str] = []

        #check if the device address makes sense
        if(not isinstance(transaction.deviceAddress, int)): 
            reasons.append("device address not an int, ")
        elif(transaction.deviceAddress < 0x8 or transaction.deviceAddress > 0x77): #address in reserved range
                if(not      (transaction.deviceAddress == 0x0 and transaction.readCount == 0)           #general addressing is fine
                    and not (transaction.deviceAddress == 0x7C or transaction.deviceAddress == 0x7F)):  #deviceID queries are fine
                    reasons.append("I2C address in invalid range, ")
        elif(transaction.deviceAddress not in self._devices):
            reasons.append("I2C transaction to unknown device, ")
            

        #check if the register address makes sense
        if(not isinstance(transaction.registerAddress, int)):
            if(not transaction.registerAddress is None):
                reasons.append("register address is an invalid type, ")
        elif(transaction.registerAddress < 0):
            reasons.append("register address is negative, ")

        #check if the write buffer is bad
        if(not transaction.writeData is None and not isinstance(transaction.writeData, bytes)):
            reasons.append("writeData is an invalid type, ")

        #check if read is bad
        if(not isinstance(transaction.readCount, int)):
            if(not transaction.readCount is None):
                reasons.append("readCount is an invalid type, ")
        elif(not transaction.readCount == 0 and not isinstance(transaction.readData, bytes)):
            reasons.append("invalid read buffer, ")

        #check if transaction is already done
        if(transaction.done.is_set()):
            reasons.append("transaction already marked as done, ")

        #return the result
        if(reasons != ""):
            return f"i2c transaction was marked invalid. Device: {transaction.deviceAddress}, Register: {transaction.registerAddress}, write_data: {transaction.writeData}, readCount = {transaction.readCount};".join(reasons)
        else:
            return None

    def _perform_transaction(self, transaction: I2CTransaction) -> None:
        """
            Execute one already-validated transaction on the physical bus.

            This method performs blocking I2C I/O.
        """

        with transaction.inProgressLock and self.dataLock: 
            write_message = None
            read_message = None

            #construct write message
            if(len(transaction.writeData) > 0 and transaction.registerAddress is not None):
                #need to send data with address first
                write_message = i2c_msg(transaction.deviceAddress, bytes(transaction.registerAddress) + transaction.writeData)
            elif(len(transaction.writeData) > 0):
                #only need to send data
                write_message = i2c_msg(transaction.deviceAddress, transaction.writeData)
            elif(transaction.registerAddress is not None):
                #only need to send an address
                write_message = i2c_msg(transaction.deviceAddress, bytes(transaction.registerAddress))

            #construct read message
            if(transaction.readCount > 0):
                read_message = i2c_msg(transaction.deviceAddress, transaction.readCount)

            #send the command
            if(write_message is not None and read_message is not None):
                self._bus.i2c_rdwr(write_message, read_message)
            elif(write_message is not None):
                self._bus.i2c_rdwr(write_message)
            elif(read_message is not None):
                self._bus.i2c_rdwr(read_message)

            #return the response
            if(read_message is not None):
                transaction.readData = read_message.buf

            transaction.done.set()

    #ensures that there is only one active driver per bus
    def __new__(cls, i2c_bus_id: int, transaction_queue: Queue[I2CTransaction]) -> Jetson_I2C_Handler:
        """Return the existing driver for ``i2c_bus_id`` or allocate one.
        """

        with cls._instance_lock:
            instance = cls._instances.get(i2c_bus_id)

            if instance is None:
                instance = super().__new__(cls)
                cls._instances[i2c_bus_id] = instance

            return instance