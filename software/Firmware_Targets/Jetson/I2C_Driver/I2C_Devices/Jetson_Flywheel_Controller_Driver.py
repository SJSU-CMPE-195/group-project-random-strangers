from __future__ import annotations

from queue import Queue
from struct import pack, unpack

from ..I2C_Handler import I2CTransaction, Jetson_I2C_Handler, I2CResult
from address_map_grabber import address_map_grabber

#get the register map from the config file
flywheelControllerRegisterMap = address_map_grabber("../../../Flywheel_Controller/config.h")

class JetsonFlywheelControllerDriver:
    def __init__(self, I2C_Handler: Jetson_I2C_Handler, I2C_Address: int):
        self._I2C_Address = I2C_Address
        self._I2C_Handler = I2C_Handler
        
    def ready(self) -> I2CResult[bool]:
        """Checks if the flywheel controller is ready to receive commands

        Returns:
            I2CResult[bool]: True if the flywheel controller is ready, False otherwise
        """
        
        def decode_ready_bit(data: bytes) -> bool:
            """Decodes the ready bit from the flywheel controller's response"""
            return data[0] == 1
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["READY"],
            readCount=1
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, decode_ready_bit)
    
    def flywheelsArmed(self) -> I2CResult[bool]:
        """Checks if the flywheel controller is armed

        Returns:
            I2CResult[bool]: True if the flywheel controller is armed, False otherwise
        """
        
        def decode_armed_bit(data: bytes) -> bool:
            """Decodes the armed bit from the flywheel controller's response"""
            return data[0] == 1
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["FLYWHEELS_ARMED"],
            readCount=1
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, decode_armed_bit)
    
    def armFlywheels(self) -> I2CResult[None]:
        """Arms the flywheel controller"""
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["FLYWHEELS_ARMED"],
            writeData=b'\x01',
            readCount=0
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction)

    def disarmFlywheels(self) -> I2CResult[None]:
        """Disarms the flywheel controller"""
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["FLYWHEELS_ARMED"],
            writeData=b'\x00',
            readCount=0
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction)

    def flywheelRPMTarget(self) -> I2CResult[float]:
        """Reads the target RPM of the flywheel controller

        Returns:
            I2CResult[float]: The target RPM of the flywheel controller
        """
        
        def decode_rpm_target(data: bytes) -> float:
            """Decodes the target RPM from the flywheel controller's response"""
            return unpack("<f", data)[0]
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["RPM_TARGET"],
            readCount=4
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, decode_rpm_target)

    def setFlywheelRPMTarget(self, rpm: float) -> I2CResult[None]:
        """Sets the target RPM of the flywheel controller"""
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["RPM_TARGET"],
            writeData=pack("<f", rpm),
            readCount=0
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction)

    def actualFlywheelRPM(self) -> I2CResult[float]:
        """Reads the actual RPM of the flywheel controller

        Returns:
            I2CResult[float]: The actual RPM of the flywheel controller
        """
        
        def decode_actual_rpm(data: bytes) -> float:
            """Decodes the actual RPM from the flywheel controller's response"""
            return unpack("<f", data)[0]
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["ACTUAL_RPM"],
            readCount=4
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, decode_actual_rpm)

    def solenoidArmed(self) -> I2CResult[bool]:
        """Checks if the solenoid is armed

        Returns:
            I2CResult[bool]: True if the solenoid is armed, False otherwise
        """
        
        def decode_armed_bit(data: bytes) -> bool:
            """Decodes the armed bit from the solenoid's response"""
            return data[0] == 1
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["SOLENOID_ARMED"],
            readCount=1
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction, decode_armed_bit)

    def armSolenoid(self) -> I2CResult[None]:
        """Arms the solenoid"""
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["SOLENOID_ARMED"],
            writeData=b'\x01',
            readCount=0
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction)

    def disarmSolenoid(self) -> I2CResult[None]:
        """Disarms the solenoid"""
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["SOLENOID_ARMED"],
            writeData=b'\x00',
            readCount=0
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction)

    def fireSolenoid(self) -> I2CResult[None]:
        """Fires the solenoid"""
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["SOLENOID_FIRE"],
            writeData=b'\x01',
            readCount=0
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction)

    def cancelSolenoidFire(self) -> I2CResult[None]:
        """Cancels or resets a solenoid fire request"""
        
        transaction = I2CTransaction(
            deviceAddress=self._I2C_Address,
            registerAddress=flywheelControllerRegisterMap["SOLENOID_FIRE"],
            writeData=b'\x00',
            readCount=0
        )
        
        self._I2C_Handler.submitTransaction(transaction)
        return I2CResult(transaction)
