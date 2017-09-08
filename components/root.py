import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
import drRigging.utils.componentUtils as componentUtils
import drRigging.objects.controls as controls
import drRigging.components.base as drBase
import drRigging.utils.metaUtils as metaUtils
reload(metaUtils)
reload(drBase)
reload(coreUtils)
reload(controls)
reload(componentUtils)

class DrRoot(drBase.DrBaseComponent):
    '''
    creates a hierarchy of controls at the world origin.
    Dedcomposes the matrix of the last child in the hierarchy and plugs its xScale into the component's global scale attribute
    '''
    def __init__(self, name, numCtrls=3, ctrlSize=10, cleanUp=1):
        super(DrRoot, self).__init__(name=name)

        self.build(numCtrls, ctrlSize)

        if cleanUp:
            self.cleanUp()

    def build(self, numCtrls, ctrlSize):
        for i in range(numCtrls):
            num = str(i+1).zfill(2)
            c = controls.circleBumpCtrl(name='%s_%s_ctrl' % (self.name, num),
                                        axis='y', radius=ctrlSize*(1-(0.15*i)))
            b = coreUtils.addParent(c, 'group', '%s_%s_buffer_srt' % (self.name, num))
            if i==0:
                b.setParent(self.interfaceGrp)
                coreUtils.colorize('green', [c])
            else:
                b.setParent(self.ctrls[-1])
                coreUtils.colorize('yellow', [c])
            self.ctrls.append(c)

    def cleanUp(self):
        super(DrRoot, self).cleanUp()

