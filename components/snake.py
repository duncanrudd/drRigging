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
        points.insert(1, coreUtils.pointsAlongVector(points[0], points[1], 2)[1])
        points.insert(numIkCtrls, coreUtils.pointsAlongVector(points[-2], points[-1], 2)[1])

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
        ikCtrls[1].getParent().setParent(ikCtrls[0])
        ikCtrls[-2].getParent().setParent(ikCtrls[-1])

        twistExtractors = []
        for i in range(len(points)-1):
            twistExtractors.append(coreUtils.isolateTwist(ikCtrls[i].worldMatrix[0], ikCtrls[i+1].worldMatrix[0], name='%s_%s' % (self.name, num), axis='x'))

        # tangent vectors
        tangentVecs = []
        tangentVecs.append(coreUtils.matrixAxisToVector(ikCtrls[0], name='%s_tangentVec_00_utl' % self.name, axis='z'))
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
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


        # create blend locator at each joint
        segMtxList = []
        segPoleVecList = []
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            segMtx = pmc.createNode('fourByFourMatrix', name='%s_segMtx_%s_utl' % (self.name, num))

            upVec = coreUtils.cross(tangentVecs[i+1].output, refVecs[i].output, name='%s_upVec_%s_utl' % (self.name, num))
            sideVec = coreUtils.cross(upVec.output, tangentVecs[i+1].output, name='%s_sideVec_%s_utl' % (self.name, num))

            sideVec.outputX.connect(segMtx.in00)
            sideVec.outputY.connect(segMtx.in01)
            sideVec.outputZ.connect(segMtx.in02)

            upVec.outputX.connect(segMtx.in10)
            upVec.outputY.connect(segMtx.in11)
            upVec.outputZ.connect(segMtx.in12)

            tangentVecs[i+1].outputX.connect(segMtx.in20)
            tangentVecs[i+1].outputY.connect(segMtx.in21)
            tangentVecs[i+1].outputZ.connect(segMtx.in22)

            d = coreUtils.isDecomposed(ikCtrls[i])
            d.outputTranslateX.connect(segMtx.in30)
            d.outputTranslateY.connect(segMtx.in31)
            d.outputTranslateZ.connect(segMtx.in32)

            segMtxList.append(segMtx)

            if i != 0:
                segMidPoint = coreUtils.blend(coreUtils.isDecomposed(ikCtrls[i-1]).outputTranslate, coreUtils.isDecomposed(ikCtrls[i+1]).outputTranslate, name='%s_segMidPoint_%s_utl' % (self.name, num))
                segMidPoint.blender.set(0.5)
                segPoleVec = coreUtils.minus([segMidPoint.output, coreUtils.isDecomposed(ikCtrls[i]).outputTranslate], name='%s_segPoleVec_%s_utl' % (self.name, num))
                segPoleVecCond = pmc.createNode('condition', name='%s_segPoleVecIsZero_%s_utl' % (self.name, num))
                segDot = coreUtils.dot(blendedTangentVecs[i].output, tangentVecs[i].output, name='%s_segDot_%s_utl' % (self.name, num))
                segDot.outputX.connect(segPoleVecCond.firstTerm)
                segPoleVecCond.secondTerm.set(1.0)
                refVecs[i].output.connect(segPoleVecCond.colorIfTrue)
                segPoleVec.output3D.connect(segPoleVecCond.colorIfFalse)
                segPoleVecNorm = coreUtils.normalizeVector(segPoleVecCond.outColor, name='%s_segPoleVecNorm_%s_utl' % (self.name, num))
                segPoleVecList.append(segPoleVecNorm)
            else:
                segPoleVecList.append(upVec)

        # create sharp matrices at each lattice division
        posYList = []
        negYList = []
        posXList = []
        negXList = []

        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            divs = latticeDivisions*2
            exceptionList = [0, 1, len(ikCtrls)-2, len(ikCtrls)-3]
            if i in exceptionList:
                divs = latticeDivisions
            isStraightCond = None
            if i < (len(ikCtrls)-2):
                isStraightCond = pmc.createNode('condition', name='%s_isStraight_%s_utl' % (self.name, num))
                tangentDot = coreUtils.dot(tangentVecs[i].output, blendedTangentVecs[i].output, name='%s_isStraightTangentDot_%s_utl' % (self.name, num))
                tangentDot.outputX.connect(isStraightCond.firstTerm)
                isStraightCond.secondTerm.set(1.0)
                segPoleVecList[i+1].output.connect(isStraightCond.colorIfTrue)
                segPoleVecList[i].output.connect(isStraightCond.colorIfFalse)

            for a in range(divs):
                num = str((i*divs)+a+1).zfill(2)
                blendVal = (1.0 / (divs))*a

                refVecBlend = coreUtils.blend(refVecs[i+1].output, refVecs[i].output, name='%s_refVecBlend_%s_utl' % (self.name, num))
                refVecBlend.blender.set(blendVal)
                refVecNorm = coreUtils.normalizeVector(refVecBlend.output, name='%s_pointRefVecNorm_%s_utl' % (self.name, num))

                pointPosBlend = coreUtils.blend(ikSrtList[i+1].outputTranslate, ikSrtList[i].outputTranslate, name='%s_pointPosBlend_%s_utl' % (self.name, num))
                pointPosBlend.blender.set(blendVal)

                pointFrontVec = coreUtils.blend(blendedTangentVecs[i+1].output, blendedTangentVecs[i].output, name='%s_pointFrontVec_%s_utl' % (self.name, num))
                pointFrontVec.blender.set(blendVal)
                pointFrontVecNorm = coreUtils.normalizeVector(pointFrontVec.output, name='%s_pointFrontVecNorm_%s_utl' % (self.name, num))

                pointUpVec = coreUtils.cross(pointFrontVecNorm.output, refVecNorm.output, name='%s_pointUpVec_%s_utl' % (self.name, num))
                pointSideVec = coreUtils.cross(pointUpVec.output, pointFrontVecNorm.output, name='%s_pointUpVec_%s_utl' % (self.name, num))

                # Calculate X and Y scaling.
                # Get dot product between front vector and tangent vector
                scaleDot = coreUtils.dot(pointFrontVec.output, tangentVecs[i+1].output, name='%s_scaleDot_%s_utl' % (self.name, num))
                scaleMult = coreUtils.divide(1.0, scaleDot.outputX, name='%s_pointScaleMult_%s_utl' % (self.name, num))


                pointXVec = None
                if isStraightCond:
                    pointXVec = coreUtils.cross(pointFrontVecNorm.output, isStraightCond.outColor, name='%s_pointPoleVec_%s_utl' % (self.name, num))
                else:
                    pointXVec = coreUtils.cross(pointFrontVecNorm.output, segPoleVecList[i].output, name='%s_pointPoleVec_%s_utl' % (self.name, num))
                pointAngle = pmc.createNode('angleBetween', name='%s_pointAngle_%s_utl' % (self.name, num))
                pointRemap = pmc.createNode('remapValue', name='%s_pointAngleRemap_%s_utl' % (self.name, num))
                pointSideVec.output.connect(pointAngle.vector1)
                pointXVec.output.connect(pointAngle.vector2)
                uc = coreUtils.convert(pointAngle.angle, 57.296, name='%s_pointAngleRadToDeg_%s_utl' % (self.name, num))
                uc.output.connect(pointRemap.inputValue)
                pointRemap.inputMax.set(180.0)
                pointRemap.outputMin.set(-1.0)
                pointAngleAbs = coreUtils.forceAbsolute(pointRemap.outValue, name='%s_pointAngleAbs_%s_utl' % (self.name, num))

                scaleXBlend = coreUtils.blend(1.0, scaleMult.outputX, name='%s_pointXScale_%s_utl' % (self.name, num), blendAttr=pointAngleAbs[1].outputX)
                scaleYBlend = coreUtils.blend(scaleMult.outputX, 1.0, name='%s_pointYScale_%s_utl' % (self.name, num), blendAttr=pointAngleAbs[1].outputX)

                pointSideVecScaled = pmc.createNode('multiplyDivide', name='%s_pointSideVecScaled_%s_utl' % (self.name, num))
                coreUtils. connectAttrToMany(scaleXBlend.outputR, [pointSideVecScaled.input1X, pointSideVecScaled.input1Y, pointSideVecScaled.input1Z])
                pointSideVec.output.connect(pointSideVecScaled.input2)

                pointUpVecScaled = pmc.createNode('multiplyDivide', name='%s_pointUpVecScaled_%s_utl' % (self.name, num))
                coreUtils.connectAttrToMany(scaleYBlend.outputR, [pointUpVecScaled.input1X, pointUpVecScaled.input1Y, pointUpVecScaled.input1Z])
                pointUpVec.output.connect(pointUpVecScaled.input2)

                pointMtx = pmc.createNode('fourByFourMatrix', name='%s_pointOrientMtx_%s_utl' % (self.name, num))

                pointSideVecScaled.outputX.connect(pointMtx.in00)
                pointSideVecScaled.outputY.connect(pointMtx.in01)
                pointSideVecScaled.outputZ.connect(pointMtx.in02)

                pointUpVecScaled.outputX.connect(pointMtx.in10)
                pointUpVecScaled.outputY.connect(pointMtx.in11)
                pointUpVecScaled.outputZ.connect(pointMtx.in12)

                pointFrontVecNorm.outputX.connect(pointMtx.in20)
                pointFrontVecNorm.outputY.connect(pointMtx.in21)
                pointFrontVecNorm.outputZ.connect(pointMtx.in22)

                pointPosBlend.outputR.connect(pointMtx.in30)
                pointPosBlend.outputG.connect(pointMtx.in31)
                pointPosBlend.outputB.connect(pointMtx.in32)

                posYList.append(coreUtils.pointMatrixMult((0, 1, 0), pointMtx.output, name='%s_pointPosY_%s_utl' % (self.name, num)))
                negYList.append(coreUtils.pointMatrixMult((0, -1, 0), pointMtx.output, name='%s_pointNegY_%s_utl' % (self.name, num)))
                posXList.append(coreUtils.pointMatrixMult((1, 0, 0), pointMtx.output, name='%s_pointPosX_%s_utl' % (self.name, num)))
                negXList.append(coreUtils.pointMatrixMult((-1, 0, 0), pointMtx.output, name='%s_pointNegX_%s_utl' % (self.name, num)))

                '''
                loc = pmc.spaceLocator()
                d = coreUtils.decomposeMatrix(pointMtx.output, name='d')
                coreUtils.connectDecomposedMatrix(d, loc)
                '''

        posYList.append(coreUtils.pointMatrixMult((0, 1, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointPosY_%s_utl' % (self.name, num)))
        negYList.append(coreUtils.pointMatrixMult((0, -1, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointNegY_%s_utl' % (self.name, num)))
        posXList.append(coreUtils.pointMatrixMult((1, 0, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointPosX_%s_utl' % (self.name, num)))
        negXList.append(coreUtils.pointMatrixMult((-1, 0, 0), ikCtrls[-1].worldMatrix[0], name='%s_pointNegX_%s_utl' % (self.name, num)))


        # create lattice
        latticeScale = math.sqrt(1*2)
        pmc.select([])
        lattice = pmc.lattice(dv=(2, 2, len(posYList)), scale=(1, 1, 1))
        lattice[0].rename('%s_lattice_def' % self.name)
        lattice[1].rename('%s_lattice_cage' % self.name)
        lattice[2].rename('%s_lattice_base' % self.name)
        lattice[2].rz.set(45)
        lattice[2].s.set((latticeScale, latticeScale, coreUtils.getDistance(start, end)))

        for i in range(len(posYList)):
            posYList[i].output.connect(lattice[1].controlPoints[(i*4)+3])
            negYList[i].output.connect(lattice[1].controlPoints[i*4])
            posXList[i].output.connect(lattice[1].controlPoints[(i*4)+1])
            negXList[i].output.connect(lattice[1].controlPoints[(i*4)+2])






