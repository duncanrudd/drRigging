import pymel.core as pmc

import drRigging.python.components.root as drRoot
reload(drRoot)

import drRigging.python.components.spine as drSpine
reload(drSpine)

import drRigging.python.utils.componentUtils as componentUtils
reload(componentUtils)




# Build root component
root = drRoot.DrRoot('root_M')


# Create some guide joints for a spine
pmc.select(None)
guideJoints = []
for i in range(10):
    num = str(i+1).zfill(2)
    j = pmc.joint(name='spine_M_%s_guide' % num)
    if i == 0:
        j.ty.set(10)
    else:
        j.ty.set(2)
    guideJoints.append(j)

# Build spine
spine = drSpine.DrSpine(name='spine_M', joints=guideJoints, numJoints=6, axis='y', upAxis='z', worldUpAxis='z', ctrlInterval=3)

# Delete guides
pmc.delete(guideJoints)

# Attach spine to root
componentUtils.connectIO(spine.ctrls[0].getParent(), root.ctrls[-1], 'spine_M')

import pymel.core as pmc

pmc.setAttr("spine_L_upVec_utl.colorIfTrue", (3, 3, 3))

import drRigging.python.utils.componentUtils as componentUtils
reload(componentUtils)

shldr = componentUtils.addCtrl(pmc.selected()[0], pmc.selected()[1], 'shldr_L', ctrlSize=6)

import drRigging.python.components.root as drRoot
reload(drRoot)

root = drRoot.DrRoot('root_M')
body = drRoot.DrRoot('body_M')

componentUtils.connectIO(pmc.selected()[0], pmc.selected()[1], 'body')

import drRigging.python.components.spine as drSpine
reload(drSpine)
spine = drSpine.DrSpine(name='spine_L', joints=pmc.selected(), numJoints=12, axis='y', upAxis='z', worldUpAxis='z', ctrlInterval=3)

import drRigging.python.components.limb as drLimb
reload(drLimb)
limb = drLimb.DrTwistyLimb(name='arm_L', joints=pmc.selected(), alignIkToJoints=0, cleanUp=1)

reload(componentUtils)
componentUtils.addSpaceSwitch(node=pmc.selected()[0], name='R', spaces=['root', 'body', 'neck'], type='rotate', ctrlNode=pmc.PyNode('wings_M_base_R_ctrl'), targetNodes=[pmc.selected()[1], pmc.selected()[2], pmc.selected()[3]])

import drRigging.python.components.twistySegment as twisty
reload(twisty)
twist = twisty.DrTwistySegment(start=pmc.selected()[0], end=pmc.selected()[1], name='twisty', numDivisions=9)

import drRigging.python.objects.fkChain as fkChain
reload(fkChain)
fkChain.fkChainBetweenPoints(pmc.selected()[0], pmc.selected()[1], name='wings_M_R', bias=0, numCtrls=5, ctrlInterval=1)

