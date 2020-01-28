import pybullet as p
from time import sleep
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf", [0,0,-2])

#boxId = p.loadURDF("cube.urdf", [0,0,4],useMaximalCoordinates = True)

cubeStartPos = [0, 0, 2]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
botId = p.loadURDF("biped/biped2d_pybullet.urdf", cubeStartPos, cubeStartOrientation)
p.changeDynamics(botId, -1, mass=0.1)

for i in range(0, p.getNumJoints(botId)):
  p.changeDynamics(botId, i, mass=0.1)
  p.setJointMotorControl2(bodyUniqueId=botId, jointIndex=i, controlMode=p.VELOCITY_CONTROL, targetVelocity = 1, force = 100)
  print(i, p.getJointInfo(botId, i))



#softId = p.loadSoftBody("torus.vtk", useNeoHookean = 1, NeoHookeanMu = 60, NeoHookeanLambda = 200, NeoHookeanDamping = 0.01, useSelfCollision = 1, frictionCoeff = 0.5)
softId = p.loadSoftBody("tube.vtk", [0, 0, 0], mass=10, useNeoHookean = 0, NeoHookeanMu = 600, NeoHookeanLambda = 200, 
                      NeoHookeanDamping = 0.01, useSelfCollision = 0, frictionCoeff = 0.5, 
                      springElasticStiffness=500, springDampingStiffness=50, springBendingStiffness=50, 
                      useMassSpring=1, useBendingSprings=1, collisionMargin=0.1)


# softId2 = p.loadSoftBody("tube.vtk", [0, 0, 2], mass=10, useNeoHookean = 0, NeoHookeanMu = 600, NeoHookeanLambda = 200, 
#                       NeoHookeanDamping = 0.01, useSelfCollision = 0, frictionCoeff = 0.5, 
#                       springElasticStiffness=500, springDampingStiffness=50, springBendingStiffness=50, 
#                       useMassSpring=1, useBendingSprings=1, collisionMargin=0.1)

#p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)
np.set_printoptions(precision=4, suppress=True)

while p.isConnected():
  p.setGravity(0,0,-10)
  x, y, z, contX, contY, contZ, contForceX, contForceY, contForceZ = p.getSoftBodyData(softId)

  print('x', len(x))
  # print('y', len(x))
  # print('z', len(x))
  print('contX', len(contX))
  # print('contY', len(contY))
  # print('contZ', len(contZ))
  # print('contForceX', len(contForceX))
  # print('contForceY', len(contForceY))
  # print('contForceZ', len(contForceZ))
  #print(contX)
  print('fx', contForceX)
  #print(contY)
  print('fy', contForceY)
  #print(contZ)
  print('fz', contForceZ)

  #for i in range(len(contForceX)):
  #  p.addUserDebugLine([contX[i], contY[i], contZ[i]], [contX[i], contY[i], contZ[i] + 3])

  #sleep(1./240.)
  p.stepSimulation()
