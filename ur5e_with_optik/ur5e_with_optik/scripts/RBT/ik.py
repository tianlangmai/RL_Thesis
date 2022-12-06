import roboticstoolbox.models.URDF as RTB_URDF

#urdf_path="/home/edip/ur5e_ws/src/ur5e_with_optik/ur5e_with_optik/urdf/ur5e_with_optik.xacro"
robot = RTB_URDF.UR5e_with_optik()
robot2 = RTB_URDF.UR5()
T = robot.fkine([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
print(robot)
print(robot2)
print(T)
T2 = robot2.fkine([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
print(T2)
sol = robot.ikine_LM(T2)
print(sol)
sol = robot2.ikine_LM(T)
print(sol)
#robot.URDF(file_path=urdf_path)
#print(robot)
#robot.plot(q=robot.qr ,backend="swift")