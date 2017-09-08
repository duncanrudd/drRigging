import drRigging.components.base as drBase
reload(drBase)
import drRigging.utils.coreUtils as coreUtils
reload(coreUtils)
import drRigging.utils.curveUtils as curveUtils
reload(curveUtils)
import drRigging.objects.controls as controls
reload(controls)
import pymel.core as pmc
import drRigging.utils.componentUtils as componentUtils
reload(componentUtils)

class DrSnake(drBase.DrBaseComponent):
    '''

    '''
    def __init__(self, start, end, name, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions=4, axis='x', upAxis='y',  cleanUp=1):
        super(DrSnake, self).__init__(name)

        self.build(start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis)

        if cleanUp:
            self.cleanUp()

    def build(self, start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis):
        start, end = coreUtils.getStartAndEnd(start, end)
        ctrlSize = coreUtils.getDistance(start, end) * .2
        radius=1

        # main control
        baseCtrl = controls.circleBumpCtrl(radius=ctrlSize,
                                           name='%s_base_ctrl' % self.name, axis=axis)
        baseSrt = coreUtils.decomposeMatrix(baseCtrl.worldMatrix[0], name='%s_baseWorldMtxToSrt_utl' % baseCtrl)
        baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        baseBuffer.setParent(self.interfaceGrp)
        self.ctrls.append(baseCtrl)

        pmc.addAttr(baseCtrl, ln='stretch', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='anchor', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='head_extend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='tail_extend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)

        # Build ik curves
        ikSoftCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls, name='%s_ik_soft_crv' % self.name)
        ikSoftCrv.setParent(self.rigGrp)

        ikHardCrv = curveUtils.curveBetweenNodes(start, end, numCVs=(numIkCtrls*3)-2, name='%s_ik_hard_crv' % self.name)
        ikHardCrv.setParent(self.rigGrp)

        # ik controls
        cvs = ikSoftCrv.getCVs(space='world')
        self.ikCtrls = []
        for i in range(numIkCtrls):
            num = str(i+1).zfill(2)
            c = controls.ballCtrl(radius=ctrlSize*.25, name='%s_ik_%s_ctrl' % (self.name, num))
            b = coreUtils.addParent(c, 'group', '%s_ik_buffer_%s_srt' % (self.name, num))
            b.t.set(cvs[i])
            b.setParent(self.interfaceGrp)
            d = coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikWorldMtxToSrt_%s_utl' % (self.name, num))
            d.outputTranslate.connect(ikSoftCrv.controlPoints[i])
            componentUtils.addSpaceSwitch(node=b, name='ik_%s' % num, spaces=['base'], type='parent', ctrlNode=c, targetNodes=[baseCtrl])
            self.ctrls.append(c)
            self.ikCtrls.append(c)

            # add attributes for hardening curves
            pmc.addAttr(c, ln='inner_sharpness', at='double', minValue=0, maxValue=1, k=1, h=0)
            pmc.addAttr(c, ln='outer_sharpness', at='double', minValue=0, maxValue=1, k=1, h=0)
            pmc.addAttr(c, ln='sharpness', at='double', minValue=0, maxValue=0.5, defaultValue=.33, k=1, h=0)

        # Connect endpoints of hard curve and soft curve
        coreUtils.connectAttrToMany(ikSoftCrv.controlPoints[0], [ikHardCrv.controlPoints[0], ikHardCrv.controlPoints[1]])
        coreUtils.connectAttrToMany(ikSoftCrv.controlPoints[numIkCtrls-1], [ikHardCrv.controlPoints[(numIkCtrls*3)-3], ikHardCrv.controlPoints[(numIkCtrls*3)-4]])

        # Connect interior controls points and set up blending for hardness
        for i in range(1, numIkCtrls-1):
            num = str(i+1).zfill(2)
            inBc = coreUtils.blend(ikSoftCrv.controlPoints[i-1], ikSoftCrv.controlPoints[i],
                                   '%s_ikInHardnessBlend_%s_utl' % (self.name, num),
                                   blendAttr=self.ikCtrls[i].sharpness)
            outBc = coreUtils.blend(ikSoftCrv.controlPoints[i+1], ikSoftCrv.controlPoints[i],
                                   '%s_ikOutHardnessBlend_%s_utl' % (self.name, num),
                                   blendAttr=self.ikCtrls[i].sharpness)

            inBc.output.connect(ikHardCrv.controlPoints[(i*3)-1])
            outBc.output.connect(ikHardCrv.controlPoints[(i*3)+1])
            ikSoftCrv.controlPoints[i].connect(ikHardCrv.controlPoints[i*3])

        '''
        # Create Lattice
        pmc.select([])
        lattice = pmc.lattice(dv=(2, 2, numIkCtrls*latticeDivisions), scale=(2, 2, coreUtils.getDistance(start, end)), ro=(0, 0, 45))
        lattice[0].rename('%s_lattice_def' % self.name)
        lattice[1].rename('%s_lattice_cage' % self.name)
        lattice[2].rename('%s_lattice_base' % self.name)
        coreUtils.align(lattice[2], lattice[1], scale=1)
        '''

        # create linear curves around each ik curve
        posYCrvSoft = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posY_soft_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % posYCrvSoft)
        pmc.move(radius, moveY=1)
        negYCrvSoft = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negY_soft_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % negYCrvSoft)
        pmc.move(radius*-1, moveY=1)
        posXCrvSoft = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posX_soft_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % posXCrvSoft)
        pmc.move(radius, moveX=1)
        negXCrvSoft = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negX_soft_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % negXCrvSoft)
        pmc.move(radius*-1, moveX=1)
        for crv in [posYCrvSoft, negYCrvSoft, posXCrvSoft, negXCrvSoft]:
            pmc.select([])
            #pmc.wire(crv, ikSoftCrv, name=crv.name().replace('crv', 'wire'), dds=[0, radius*100.0])

        posYCrvSharp = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posY_sharp_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % posYCrvSharp)
        pmc.move(radius, moveY=1)
        negYCrvSharp = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negY_sharp_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % negYCrvSharp)
        pmc.move(radius*-1, moveY=1)
        posXCrvSharp = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posX_sharp_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % posXCrvSharp)
        pmc.move(radius, moveX=1)
        negXCrvSharp = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negX_sharp_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % negXCrvSharp)
        pmc.move(radius*-1, moveX=1)
        for crv in [posYCrvSharp, negYCrvSharp, posXCrvSharp, negXCrvSharp]:
            pmc.select([])
            #pmc.wire(crv, ikHardCrv, name=crv.name().replace('crv', 'wire'), dds=[0, radius*100.0])

        # create linear curves to drive the lattice
        posYCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posY_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % posYCrv)
        pmc.move(radius, moveY=1)
        negYCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negY_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % negYCrv)
        pmc.move(radius*-1, moveY=1)
        posXCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_posX_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % posXCrv)
        pmc.move(radius, moveX=1)
        negXCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls*latticeDivisions, name='%s_negX_crv' % self.name, degree=1)
        pmc.select('%s.cv[ * ]' % negXCrv)
        pmc.move(radius*-1, moveX=1)

        # point on curve info nodes for measuring curvature at joints
        infoNodes = curveUtils.sampleCurve(crv=ikHardCrv, numSamples=numIkCtrls, name='ikCrv')

        # create vectors from info nodes to lattice points and from info nodes to curvature centres
        latticeVecsY = []
        latticeVecsX = []
        for i in range(numIkCtrls):
            num = str(i+1).zfill(2)
            pmaY = coreUtils.minus([posYCrvSharp.controlPoints[i*latticeDivisions], infoNodes[i].result.position], name='%s_vecY_%s_utl' % (self.name, num))
            vpY = coreUtils.normalizeVector(pmaY.output3D, name='%s_vecYNormalized_%s_utl' % (self.name, num))

            pmaX = coreUtils.minus([posXCrvSharp.controlPoints[i*latticeDivisions], infoNodes[i].result.position], name='%s_vecX_%s_utl' % (self.name, num))
            vpX = coreUtils.normalizeVector(pmaX.output3D, name='%s_vecXNormalized_%s_utl' % (self.name, num))

            pmaC = coreUtils.minus([infoNodes[i].result.curvatureCenter, infoNodes[i].result.position], name='%s_vecX_%s_utl' % (self.name, num))
            curvatureCond = pmc.createNode('condition', name='%s_isCurved_%s_utl' % (self.name, num))
            infoNodes[i].curvatureRadius.connect(curvatureCond.firstTerm)
            curvatureCond.colorIfTrue.set((0, 1, 0))
            pmaC.output3D.connect(curvatureCond.colorIfFalse)
            vpC = coreUtils.normalizeVector(curvatureCond.outColor, name='%s_vecYNormalized_%s_utl' % (self.name, num))

            dotY = coreUtils.dot(vpC.output, vpY.output, name='%s_dotY_%s_utl' % (self.name, num))
            dotYRemap = pmc.createNode('remapValue', name='%s_dotYNormalized_%s_utl' % (self.name, num))
            dotY.outputX.connect(dotYRemap.inputValue)
            dotYRemap.inputMin.set(-1)
            dotYRev = coreUtils.reverse(dotYRemap.outValue, name='%s_dotYReversed_%s_utl' % (self.name, num))

            dotX = coreUtils.dot(vpC.output, vpX.output, name='%s_dotX_%s_utl' % (self.name, num))
            dotXRemap = pmc.createNode('remapValue', name='%s_dotXNormalized_%s_utl' % (self.name, num))
            dotX.outputX.connect(dotXRemap.inputValue)
            dotXRemap.inputMin.set(-1)
            dotXRev = coreUtils.reverse(dotYRemap.outValue, name='%s_dotXReversed_%s_utl' % (self.name, num))

            innerMultY = coreUtils.multiply(dotYRemap.outValue, self.ikCtrls[i].inner_sharpness, name='%s_innerSharpenY_%s_utl' % (self.name, num))
            outerMultY = coreUtils.multiply(dotYRev.outputX, self.ikCtrls[i].outer_sharpness, name='%s_outerSharpenY_%s_utl' % (self.name, num))
            sharpenPosY = coreUtils.add([innerMultY.outputX, outerMultY.outputX], name='%s_sharpenPosY_%s_utl' % (self.name, num))
            sharpenNegY = coreUtils.convert(sharpenPosY.output1D, -1.0, name='%s_sharpenNegY_%s_utl' % (self.name, num))

            innerMultX = coreUtils.multiply(dotXRemap.outValue, self.ikCtrls[i].inner_sharpness, name='%s_innerSharpenX_%s_utl' % (self.name, num))
            outerMultX = coreUtils.multiply(dotXRev.outputX, self.ikCtrls[i].outer_sharpness, name='%s_outerSharpenX_%s_utl' % (self.name, num))
            sharpenPosX = coreUtils.add([innerMultX.outputX, outerMultX.outputX], name='%s_sharpenPosX_%s_utl' % (self.name, num))
            sharpenNegX = coreUtils.convert(sharpenPosX.output1D, -1.0, name='%s_sharpenNegX_%s_utl' % (self.name, num))

            bcPosY = coreUtils.blend(posYCrvSoft.controlPoints[i], posYCrvSharp.controlPoints[i], name='%s_blendPosY_%s_utl' % (self.name, num), blendAttr=sharpenPosY.output1D)
            bcPosY.output.connect(posYCrv.controlPoints[i])

            bcNegY = coreUtils.blend(negYCrvSoft.controlPoints[i], negYCrvSharp.controlPoints[i], name='%s_blendNegY_%s_utl' % (self.name, num), blendAttr=sharpenNegY.output)
            bcNegY.output.connect(negYCrv.controlPoints[i])

            bcPosX = coreUtils.blend(posXCrvSoft.controlPoints[i], posXCrvSharp.controlPoints[i], name='%s_blendPosX_%s_utl' % (self.name, num), blendAttr=sharpenPosX.output1D)
            bcPosX.output.connect(posXCrv.controlPoints[i])

            bcNegX = coreUtils.blend(negXCrvSoft.controlPoints[i], negXCrvSharp.controlPoints[i], name='%s_blendNegX_%s_utl' % (self.name, num), blendAttr=sharpenNegX.output)
            bcNegX.output.connect(negXCrv.controlPoints[i])




    def cleanUp(self):
        pass
        #super(DrSnake, self).cleanUp()
