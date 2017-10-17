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

import math

class DrSnake(drBase.DrBaseComponent):
    '''
    '''
    def __init__(self, start, end, name, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions=4, axis='x', upAxis='y',  cleanUp=1):
        super(DrSnake, self).__init__(name)

        self.build(start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis)

        if cleanUp:
            # self.cleanUp()
            pass

    def build(self, start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis):
        start, end = coreUtils.getStartAndEnd(start, end)
        ctrlSize = coreUtils.getDistance(start, end) * .1
        radius=1

        # main control
        baseCtrl = controls.circleBumpCtrl(radius=ctrlSize,
                                           name='%s_base_ctrl' % self.name, axis=axis)
        baseSrt = coreUtils.decomposeMatrix(baseCtrl.worldMatrix[0], name='%s_baseWorldMtxToSrt_utl' % baseCtrl)
        baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        baseBuffer.setParent(self.interfaceGrp)
        self.ctrls.append(baseCtrl)

        points = coreUtils.pointsAlongVector(start, end, divisions=numIkCtrls-1)

        # ik ctrls
        ikCtrls=[]
        refVecs = []
        ikSrtList = []
        for i in range(len(points)):
            num = str(i+1).zfill(2)

            c = controls.ballCtrl(radius=ctrlSize*.5, name='%s_ik_%s_ctrl' % (self.name, num))
            b = coreUtils.addParent(c, 'group', '%s_ik_%s_buffer_srt' % (self.name, num))
            b.setParent(self.interfaceGrp)
            b.t.set(points[i])
            ikCtrls.append(c)
            refVecs.append(coreUtils.matrixAxisToVector(c, name='%s_referenceVec_%s_utl' % (self.name, num), axis='x'))
            ikSrtList.append(coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikCtrlMtxToSrt_%s_utl' % (self.name, num)))

            # Add attributes for blending between soft and sharp points
            pmc.addAttr(c, ln='soften_outer', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
            pmc.addAttr(c, ln='soften_inner', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
            pmc.addAttr(c, ln='bend_pos_x', at='double', k=1, h=0)
            pmc.addAttr(c, ln='bend_neg_x', at='double', k=1, h=0)
            pmc.addAttr(c, ln='bend_pos_y', at='double', k=1, h=0)
            pmc.addAttr(c, ln='bend_neg_y', at='double', k=1, h=0)

        # tangent vectors
        tangentVecs = []
        tangentVecs.append(coreUtils.matrixAxisToVector(ikCtrls[0], name='%s_tangentVec_00_utl' % self.name, axis='z'))
        for i in range(len(ikCtrls)-1):
            num = str(i+2).zfill(2)
            vec = coreUtils.vectorBetweenNodes(ikCtrls[i], ikCtrls[i+1], name='%s_tangentVec_%s_utl' % (self.name, num))
            vecNorm = coreUtils.normalizeVector(vec.output3D, name='%s_tangentVecNorm_%s_utl' % (self.name, num))
            tangentVecs.append(vecNorm)

        tangentVecs.append(coreUtils.matrixAxisToVector(ikCtrls[-1], name='%s_tangentVec_%s_utl' % (self.name, str(len(points)).zfill(2)), axis='z'))

        blendedTangentVecs=[]
        # create blended tangent vector
        for i in range(len(ikCtrls)):
            num = str(i+1).zfill(2)
            bc = coreUtils.blend(tangentVecs[i].output, tangentVecs[i+1].output, name='%s_blendedTangentVec_%s_utl' % (self.name, num))
            bc.blender.set(0.5)
            blendedTangentVecNorm = coreUtils.normalizeVector(bc.output, name='%s_blendedTangentVecNorm_%s_utl' % (self.name, num))
            blendedTangentVecs.append(blendedTangentVecNorm)
        blendedTangentVecs.append(tangentVecs[-1])


        # create matrix and pole vector for each joint
        segMtxList = []
        segPoleVecList = []
        segDots = []
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            segMtx = pmc.createNode('fourByFourMatrix', name='%s_segMtx_%s_utl' % (self.name, num))

            upVec = coreUtils.cross(blendedTangentVecs[i].output, refVecs[i].output, name='%s_upVec_%s_utl' % (self.name, num))
            sideVec = coreUtils.cross(upVec.output, blendedTangentVecs[i].output, name='%s_sideVec_%s_utl' % (self.name, num))

            segDot = coreUtils.dot(blendedTangentVecs[i].output, tangentVecs[i+1].output, name='%s_segDot_%s_utl' % (self.name, num))
            segDots.append(segDot)

            if i != 0:
                segMidPoint = coreUtils.blend(coreUtils.isDecomposed(ikCtrls[i-1]).outputTranslate, coreUtils.isDecomposed(ikCtrls[i+1]).outputTranslate, name='%s_segMidPoint_%s_utl' % (self.name, num))
                segMidPoint.blender.set(0.5)
                segPoleVec = coreUtils.minus([segMidPoint.output, coreUtils.isDecomposed(ikCtrls[i]).outputTranslate], name='%s_segPoleVec_%s_utl' % (self.name, num))
                segPoleVecCond = pmc.createNode('condition', name='%s_segPoleVecIsZero_%s_utl' % (self.name, num))
                segDot.outputX.connect(segPoleVecCond.firstTerm)
                segPoleVecCond.secondTerm.set(1.0)
                refVecs[i].output.connect(segPoleVecCond.colorIfTrue)
                segPoleVec.output3D.connect(segPoleVecCond.colorIfFalse)
                segPoleVecNorm = coreUtils.normalizeVector(segPoleVecCond.outColor, name='%s_segPoleVecNorm_%s_utl' % (self.name, num))
                segPoleVecList.append(segPoleVecNorm)
            else:
                segPoleVecList.append(upVec)

            # construct segment matrix
            sideVec.outputX.connect(segMtx.in00)
            sideVec.outputY.connect(segMtx.in01)
            sideVec.outputZ.connect(segMtx.in02)

            upVec.outputX.connect(segMtx.in10)
            upVec.outputY.connect(segMtx.in11)
            upVec.outputZ.connect(segMtx.in12)

            blendedTangentVecs[i].outputX.connect(segMtx.in20)
            blendedTangentVecs[i].outputY.connect(segMtx.in21)
            blendedTangentVecs[i].outputZ.connect(segMtx.in22)

            d = coreUtils.isDecomposed(ikCtrls[i])
            d.outputTranslateX.connect(segMtx.in30)
            d.outputTranslateY.connect(segMtx.in31)
            d.outputTranslateZ.connect(segMtx.in32)

            segMtxList.append(segMtx)

        posYPoints = []
        negYPoints = []
        posXPoints = []
        negXPoints = []

        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)

            isStraightCond = None
            if i < (len(ikCtrls)-2):
                isStraightCond = pmc.createNode('condition', name='%s_isStraight_%s_utl' % (self.name, num))
                segDot.outputX.connect(isStraightCond.firstTerm)
                isStraightCond.secondTerm.set(1.0)
                segPoleVecList[i+1].output.connect(isStraightCond.colorIfTrue)
                segPoleVecList[i].output.connect(isStraightCond.colorIfFalse)

            # Calculate X and Y scaling.
            scaleMult = coreUtils.divide(1.0, segDots[i].outputX, name='%s_segScaleMult_%s_utl' % (self.name, num))

            # Measure tangent vec relative to first mtx of segment so we know direction of bend
            worldTangentVec = coreUtils.add([tangentVecs[i+1].output, ikSrtList[i].outputTranslate], name='%s_seg_%s_worldTangentVec_utl' % (self.name, num))
            segMtxInverse = coreUtils.inverseMatrix(segMtxList[i].output, name='%s_seg_%s_inverseMtx_utl' % (self.name, num))
            localTangentVec = coreUtils.pointMatrixMult(worldTangentVec.output3D, segMtxInverse.outputMatrix, name='%s_seg_%s_localTangentVec_utl' % (self.name, num))
            localProjectVec = pmc.createNode('vectorProduct', name='%s_seg_%s_localProjectVec_utl' % (self.name, num))
            localProjectVec.input1Z.set(.0001)
            localProjectVec.operation.set(0)
            localProjectVec.normalizeOutput.set(1)
            localYVecAbs = coreUtils.forceAbsolute(localProjectVec.outputY, name='%s_seg_%s_localYVecAbs' % (self.name, num))[1]
            localXVecAbs = coreUtils.forceAbsolute(localProjectVec.outputX, name='%s_seg_%s_localXVecAbs' % (self.name, num))[1]
            localTangentVec.outputX.connect(localProjectVec.input1X)
            localTangentVec.outputY.connect(localProjectVec.input1Y)

            posYRemap = pmc.createNode('remapValue', name='%s_seg_%s_posYRemap_utl' % (self.name, num))
            localProjectVec.outputY.connect(posYRemap.inputValue)

            negYRemap = pmc.createNode('remapValue', name='%s_seg_%s_negYRemap_utl' % (self.name, num))
            negYInvert = coreUtils.convert(localProjectVec.outputY, -1, name='%s_seg_%s_negYInvert_utl' % (self.name, num))
            negYInvert.output.connect(negYRemap.inputValue)

            posXRemap = pmc.createNode('remapValue', name='%s_seg_%s_posXRemap_utl' % (self.name, num))
            localProjectVec.outputX.connect(posXRemap.inputValue)

            negXRemap = pmc.createNode('remapValue', name='%s_seg_%s_negXRemap_utl' % (self.name, num))
            negXInvert = coreUtils.convert(localProjectVec.outputX, -1, name='%s_seg_%s_negXInvert_utl' % (self.name, num))
            negXInvert.output.connect(negXRemap.inputValue)

            posYOuterMult = coreUtils.multiply(posYRemap.outValue, ikCtrls[i].soften_outer, name='%s_seg_%s_posYOuterMult_utl' % (self.name, num))
            posYInnerMult = coreUtils.multiply(negYRemap.outValue, ikCtrls[i].soften_inner, name='%s_seg_%s_posYInnerMult_utl' % (self.name, num))
            posYTotal = coreUtils.add([posYOuterMult.outputX, posYInnerMult.outputX], name='%s_seg_%s_posYBlendAmount_utl' % (self.name, num))
            posYTotal.output1D.connect(ikCtrls[i].bend_pos_y)

            negYOuterMult = coreUtils.multiply(posYRemap.outValue, ikCtrls[i].soften_inner, name='%s_seg_%s_negYOuterMult_utl' % (self.name, num))
            negYInnerMult = coreUtils.multiply(negYRemap.outValue, ikCtrls[i].soften_outer, name='%s_seg_%s_negYInnerMult_utl' % (self.name, num))
            negYTotal = coreUtils.add([negYOuterMult.outputX, negYInnerMult.outputX], name='%s_seg_%s_negYBlendAmount_utl' % (self.name, num))
            negYTotal.output1D.connect(ikCtrls[i].bend_neg_y)

            posXOuterMult = coreUtils.multiply(posXRemap.outValue, ikCtrls[i].soften_outer, name='%s_seg_%s_posXOuterMult_utl' % (self.name, num))
            posXInnerMult = coreUtils.multiply(negXRemap.outValue, ikCtrls[i].soften_inner, name='%s_seg_%s_posXInnerMult_utl' % (self.name, num))
            posXTotal = coreUtils.add([posXOuterMult.outputX, posXInnerMult.outputX], name='%s_seg_%s_posXBlendAmount_utl' % (self.name, num))
            posXTotal.output1D.connect(ikCtrls[i].bend_pos_x)

            negXOuterMult = coreUtils.multiply(posXRemap.outValue, ikCtrls[i].soften_inner, name='%s_seg_%s_negXOuterMult_utl' % (self.name, num))
            negXInnerMult = coreUtils.multiply(negXRemap.outValue, ikCtrls[i].soften_outer, name='%s_seg_%s_negXInnerMult_utl' % (self.name, num))
            negXTotal = coreUtils.add([negXOuterMult.outputX, negXInnerMult.outputX], name='%s_seg_%s_negXBlendAmount_utl' % (self.name, num))
            negXTotal.output1D.connect(ikCtrls[i].bend_neg_x)

            posYBlend = coreUtils.blend(scaleMult.outputX, 1.0, name='%s_seg_%s_posYBlend_utl' % (self.name, num), blendAttr=localYVecAbs.outputX)
            negYInvert = coreUtils.convert(posYBlend.outputR, -1.0, name='%s_seg_%s_negYLength_utl' % (self.name, num))

            posXBlend = coreUtils.blend(scaleMult.outputX, 1.0, name='%s_seg_%s_posXBlend_utl' % (self.name, num), blendAttr=localXVecAbs.outputX)
            negXInvert = coreUtils.convert(posXBlend.outputR, -1.0, name='%s_seg_%s_negXLength_utl' % (self.name, num))

            posYPoint = coreUtils.pointMatrixMult((0, 0, 0), segMtxList[i].output, name='%s_seg_%s_posYPoint_utl' % (self.name, num))
            posYBlend.outputR.connect(posYPoint.input1Y)
            posYPoints.append(posYPoint)

            negYPoint = coreUtils.pointMatrixMult((0, 0, 0), segMtxList[i].output, name='%s_seg_%s_negYPoint_utl' % (self.name, num))
            negYInvert.output.connect(negYPoint.input1Y)
            negYPoints.append(negYPoint)

            posXPoint = coreUtils.pointMatrixMult((0, 0, 0), segMtxList[i].output, name='%s_seg_%s_posXPoint_utl' % (self.name, num))
            posXBlend.outputR.connect(posXPoint.input1X)
            posXPoints.append(posXPoint)

            negXPoint = coreUtils.pointMatrixMult((0, 0, 0), segMtxList[i].output, name='%s_seg_%s_negXPoint_utl' % (self.name, num))
            negXInvert.output.connect(negXPoint.input1X)
            negXPoints.append(negXPoint)



        # create lattice
        latticeScale = math.sqrt(1*2)
        pmc.select([])
        lattice = pmc.lattice(dv=(2, 2, len(posYPoints)), scale=(1, 1, 1))
        lattice[0].rename('%s_lattice_def' % self.name)
        lattice[1].rename('%s_lattice_cage' % self.name)
        lattice[2].rename('%s_lattice_base' % self.name)
        lattice[2].rz.set(45)
        lattice[2].s.set((latticeScale, latticeScale, coreUtils.getDistance(start, end)))

        for i in range(len(posYPoints)):
            posYPoints[i].output.connect(lattice[1].controlPoints[(i*4)+3])
            negYPoints[i].output.connect(lattice[1].controlPoints[i*4])
            posXPoints[i].output.connect(lattice[1].controlPoints[(i*4)+1])
            negXPoints[i].output.connect(lattice[1].controlPoints[(i*4)+2])








