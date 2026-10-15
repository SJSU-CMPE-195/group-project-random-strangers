from __future__ import annotations
import threading
from dataclasses import dataclass, field
from queue import Empty, Queue
from time import time, monotonic
from typing import ClassVar
import logging

from smbus2 import SMBus, i2c_msg

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

@dataclass
class I2CTransaction:
    deviceAddress: int = 0
    registerAddress: int = -1

    writeData: bytes = bytes(0)

    readCount: int = 0
    readData: bytearray = field(default_factory=bytearray)

    inProgressLock: threading.Lock = field(default_factory=threading.Lock)
    done: threading.Event = field(default=threading.Event())
    

class Jetson_I2C_Driver:
    # One driver instance per I2C bus
    _error_check_period_s: ClassVar[float] = 0.25
    _instances: ClassVar[dict[int, Jetson_I2C_Driver]] = {}
    _instance_lock: ClassVar[threading.Lock] = threading.Lock()

    def __init__(self, i2c_bus_id: int, transaction_queue: Queue[I2CTransaction | None], doErrorChecking: bool = False) -> None:
        #prevent init from resetting an existing instance.
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
                self.runThread: threading.Thread = threading.Thread(target=self.runner_task)

                logger.debug(f"Initialized an I2C Driver on BusID:{i2c_bus_id} with error checking {"enabled" if doErrorChecking else "disabled"}")
                self._initialized = True

            except OSError:
                #remove failed instances
                self._instances.pop(i2c_bus_id, None)
                raise

    def close(self) -> None:
        """Close the bus and allow a new instance to be created."""
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
            if(i2cAddress > 7 and i2cAddress < 120 and i2cAddress not in self._devices): #do not allow reserved addresses or device conflicts
    
                with self.dataLock:
                    self._devices[i2cAddress] = (device_name, errors_supported)
    
                return True
    
            return False

    def runner_task(self) -> None:
        next_error_check = monotonic()

        while(True):
            if self.doErrorChecking:
                if(monotonic() >= next_error_check):
                    self.errors = self._read_errors()
                    self.lastErrorCheck = time()
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
                        self._perform_transaction(next_transaction)
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
                        self._perform_transaction(next_transaction)
                        next_transaction.done.set()
                    else:
                        logging.warning(error_reason)

    def _validate_transaction(self, transaction: I2CTransaction) -> str | None: 
        """
            validates a transaction to ensure it is well formed and can be sent over I2C

            returns error reason if bad, None if good
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
        elif(not transaction.readCount == 0 and not isinstance(transaction.readData, bytearray)):
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
            fulfils a transaction over I2C

            assumes transaction has already been validated
        """

        with transaction.inProgressLock: 
            write_message = None
            read_message = None

            #construct write message
            if(len(transaction.writeData) > 0 and transaction.registerAddress >= 0):
                #need to send data with address first
                write_message = i2c_msg(transaction.deviceAddress, bytes(transaction.registerAddress) + transaction.writeData)
            elif(len(transaction.writeData) > 0):
                #only need to send data
                write_message = i2c_msg(transaction.deviceAddress, transaction.writeData)
            elif(transaction.registerAddress >= 0):
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
                transaction.readData.append(read_message.buf)

            transaction.done.set()

    def _read_errors(self) -> dict[str, list[str]]:
        """assumes i2c devices follow the standard error protocol of 
            0x1 == master error
            0x2-0x5 == error bitset
            0x6 == selected_error_id
            0x7 == error length
            0x8 == error
        """
        all_new_errors: dict[str, list[str]] = dict()
        with self.dataLock:
            for device_address, (device_name, errors_supported) in self._devices.items():
                if(errors_supported == False):
                    continue

                #querry errors
                error_metadata: list[int] = self._bus.read_i2c_block_data(device_address, 0x1, 5)
                error_integer = int.from_bytes(error_metadata[1:4], byteorder="big", signed=False)

                if(error_metadata[0] != 0): #error(s) detected
                    device_errors: list[str] = list()

                    #loop through each set error bit
                    while(error_integer > 0):
                        bit_pos = (error_integer & error_integer).bit_length() - 1

                        #command the device to provide the error corresponding to the set bit
                        self._bus.write_byte_data(device_address, 0x6, bit_pos)

                        #get the error string reads a number of bytes = to the first read value
                        error_data = self._bus.read_block_data(device_address, 0x7)

                        device_errors.append("".join(chr(byte) for byte in error_data))

                    all_new_errors[device_name] = device_errors

        return all_new_errors

    #ensures that there is only one active driver per bus
    def __new__(cls, i2c_bus_id: int, transaction_queue: Queue[I2CTransaction]) -> Jetson_I2C_Driver:
        with cls._instance_lock:
            instance = cls._instances.get(i2c_bus_id)

            if instance is None:
                instance = super().__new__(cls)
                cls._instances[i2c_bus_id] = instance

            return instance