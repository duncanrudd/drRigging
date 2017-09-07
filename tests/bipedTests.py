import sys
thePath = 'D:\\TomMelson\\workspace\\'
if not thePath in sys.path:
    sys.path.append(thePath)
import drRigging
reload(drRigging)

import pymel.core as pmc

import drRigging.python.components.root as drRoot
reload(drRoot)

import drRigging.python.components.spine as drSpine
reload(drSpine)

import drRigging.python.utils.componentUtils as componentUtils
reload(componentUtils)


root = drRoot.DrRoot('root_M')
#Scale it
# Create joints
spine = drSpine.DrSpine(name='spine_M', joints=pmc.selected(), numJoints=6, axis='y', upAxis='z', worldUpAxis='z', ctrlInterval=1)
# Select root of spine (buff) then smallest root control
pmc.delete(_JOINTS_)
componentUtils.connectIO(spine.ctrls[0].getParent(), root.ctrls[-1], 'root')
# Create locator and place for shoulder collar bone) select spine IK ctrl then loc
shldr = componentUtils.addCtrl(pmc.selected()[0], pmc.selected()[1], 'shldr_L', ctrlSize=6)
# Add 4 arm guide joints
import drRigging.python.components.limb as drLimb
reload(drLimb)
limb = drLimb.DrTwistyLimb(name='arm_L', joints=pmc.selected(), alignIkToJoints=0, cleanUp=1)

componentUtils.connectIO(pmc.selected()[0], pmc.selected()[1], 'shldr')
# Select buff of arm IK ctrl, then body sub ctrl (inner of the 3) & root sub ctrl
componentUtils.addSpaceSwitch(node=pmc.selected()[0], name='ik', spaces=['root', 'body'], type='parent', ctrlNode=pmc.PyNode('arm_L_ik_ctrl'), targetNodes=[pmc.selected()[1], pmc.selected()[2]])

# Template for guide, adjust by user then build rig and nuke guide
