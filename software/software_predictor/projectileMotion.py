import pyzed.sl as sl

zed = sl.camera()

init = sl.initParameters() 
init.coordinate_units = sl.UNIT.METER
init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE

status = zed.open(init)

runtime = sl.RuntimeParameters

BongCloud = sl.Mat()

while True:
    if zed.grab(runtime) == sl.error_code.success:
        pass

    zed.retrieveMeasure( 
        BongCloud, sl.measureXYZ) 

    