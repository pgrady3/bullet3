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
                      useMassSpring=1, useBendingSprings=1, collisionMargin=0.05)


# softId2 = p.loadSoftBody("tube.vtk", [0, 0, 2], mass=10, useNeoHookean = 0, NeoHookeanMu = 600, NeoHookeanLambda = 200, 
#                       NeoHookeanDamping = 0.01, useSelfCollision = 0, frictionCoeff = 0.5, 
#                       springElasticStiffness=500, springDampingStiffness=50, springBendingStiffness=50, 
#                       useMassSpring=1, useBendingSprings=1, collisionMargin=0.1)

p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)
np.set_printoptions(precision=4, suppress=True)

debug_lines = []
for i in range(100):
    line_id = p.addUserDebugLine([0,0,0], [0,0,0])
    debug_lines.append(line_id)

while p.isConnected():
  p.setGravity(0,0,-10)
  x, y, z, contX, contY, contZ, contForceX, contForceY, contForceZ = p.getSoftBodyData(softId)

  contact_pt = np.stack((contX, contY, contZ)).T
  contact_force = np.stack((contForceX, contForceY, contForceZ)).T
  #print(contact_pt.shape)

  print('num nodes', len(x), 'contact nodes', len(contX))
  #print('fx', contForceX)
  #print('fy', contForceY)
  #print('fz', contForceZ)

  for i in range(len(debug_lines)):
    if i < len(contX):
      debug_lines[i] = p.addUserDebugLine(contact_pt[i, :], contact_pt[i, :] + contact_force[i, :], lineWidth=3, replaceItemUniqueId=debug_lines[i])
    else:

      debug_lines[i] = p.addUserDebugLine([0,0,0], [0,0,0], replaceItemUniqueId=debug_lines[i])

  #sleep(1./240.)
  p.stepSimulation()
