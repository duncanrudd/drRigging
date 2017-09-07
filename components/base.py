import drRigging.python.utils.coreUtils as coreUtils
import pymel.core as pmc

reload(coreUtils)

class DrBaseComponent(object):
    def __init__(self, name):
        #super(DrBaseComponent, self).__init__()

        self.name = name
        self.ctrls = []

        # Create base hierarchy for component
        self.mainGrp = pmc.group(empty=1, name='%s_cmpnt' % self.name)
        self.inputGrp = coreUtils.addChild(self.mainGrp, 'group', '%s_input' % self.name)
        self.interfaceGrp = coreUtils.addChild(self.mainGrp, 'group', '%s_interface' % self.name)
        self.outputGrp = coreUtils.addChild(self.mainGrp, 'group', '%s_output' % self.name)
        self.rigGrp = coreUtils.addChild(self.mainGrp, 'group', '%s_rig' % self.name)
        self.deformGrp = coreUtils.addChild(self.mainGrp, 'group', '%s_deform' % self.name)
        self.subGrp = coreUtils.addChild(self.mainGrp, 'group', '%s_subCmpnt' % self.name)
        pmc.addAttr(self.mainGrp, ln='globalScale', at='double')
        self.mainGrp.globalScale.set(1)

    def build(self):
        '''
        override this function in inheriting classes to build your components
        :return:
        '''
        pass
    def cleanUp(self):
        '''
        hides rig and deform groups. Override in child classes and call explicitly from within overriding function
        locks and hides visibility of controls and arnold attributes on shapes of controls
        :return:
        '''
        self.rigGrp.visibility.set(0)
        self.deformGrp.visibility.set(0)
        coreUtils.attrCtrl(nodeList=self.ctrls, attrList=['visibility'])

        for c in self.ctrls:
            shapes = pmc.listRelatives(c, s=1)
            for shape in shapes:
                attrList = ['aiRenderCurve', 'aiCurveWidth', 'aiSampleRate',
                            'aiCurveShaderR', 'aiCurveShaderG', 'aiCurveShaderB']
                for attr in attrList:
                    if not pmc.attributeQuery(attr, node=shape, ex=1):
                        attrList.remove(attr)
                coreUtils.attrCtrl(nodeList=[shape], attrList=attrList)

