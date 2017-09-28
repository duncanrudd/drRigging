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
        for i in range(len(points)):
            num = str(i+1).zfill(2)

            c = controls.ballCtrl(radius=ctrlSize*.5, name='%s_ik_%s_ctrl' % (self.name, num))
            b = coreUtils.addParent(c, 'group', '%s_ik_%s_buffer_srt' % (self.name, num))
            b.setParent(self.interfaceGrp)
            b.t.set(points[i])
            ikCtrls.append(c)
            refVecs.append(coreUtils.matrixAxisToVector(c, name='%s_referenceVec_%s_utl' % (self.name, num), axis='x'))
        ikCtrls[1].getParent().setParent(ikCtrls[0])
        ikCtrls[-2].getParent().setParent(ikCtrls[-1])


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

        # create sharp matrices at each lattice division
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            divs = latticeDivisions*2
            exceptionList = [0, 1, len(ikCtrls)-2, len(ikCtrls)-3]
            if i in exceptionList:
                divs = latticeDivisions
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

            d = coreUtils.decomposeMatrix(ikCtrls[i].worldMatrix[0], name='%s_segMtxToSrt_%s_utl' % (self.name, num))
            d.outputTranslateX.connect(segMtx.in30)
            d.outputTranslateY.connect(segMtx.in31)
            d.outputTranslateZ.connect(segMtx.in32)

            dist = coreUtils.distanceBetweenNodes(ikCtrls[i], ikCtrls[i+1], name='%s_segLength_%s_utl' % (self.name, num))

            for a in range(divs):
                num = str((i*divs)+a+1).zfill(2)
                blendVal = (1.0 / (divs))*a

                mtx = None
                d = None
                if a == 0:
                    mtx = segMtx
                    d = coreUtils.decomposeMatrix(mtx.output, name='%s_pointMtxToSrt_%s_utl' % (self.name, num))
                else:
                    pointLocalOffset = coreUtils.convert(dist.distance, blendVal, name='%s_pointSegOffset_%s_utl' % (self.name, num))
                    localMtx = pmc.createNode('fourByFourMatrix', name='%s_pointLocalMtx_%s_utl' % (self.name, num))
                    pointLocalOffset.output.connect(localMtx.in32)
                    mtx = coreUtils.multiplyMatrices([localMtx.output, segMtx.output], name='%s_pointMtx_%s_utl' % (self.name, num))
                    d = coreUtils.decomposeMatrix(mtx.matrixSum, '%s_pointMtxToSrt_%s_utl' % (self.name, num))

                # Create a locator the orients according to blended tangents
                loc = coreUtils.addChild(self.rigGrp, 'locator', name='%s_pointOrient_%s_srt' % (self.name, num))

                pointFrontVec = coreUtils.blend(blendedTangentVecs[i+1].output, blendedTangentVecs[i].output, name='%s_pointFrontVec_%s_utl' % (self.name, num))
                pointFrontVec.blender.set(blendVal)
                pointFrontVecNorm = coreUtils.normalizeVector(pointFrontVec.output, name='%s_pointFrontVecNorm_%s_utl' % (self.name, num))

                pointRefVec = coreUtils.blend(refVecs[i+1].output, refVecs[i].output, name='%s_pointRefVec_%s_utl' % (self.name, num))
                pointRefVec.blender.set(blendVal)
                pointRefVecNorm = coreUtils.normalizeVector(pointRefVec.output, name='%s_pointReferenceVecNorm_%s_utl' % (self.name, num))

                pointUpVec = coreUtils.cross(pointFrontVecNorm.output, pointRefVecNorm.output, name='%s_pointUpVec_%s_utl' % (self.name, num))
                pointSideVec = coreUtils.cross(pointUpVec.output, pointFrontVecNorm.output, name='%s_pointUpVec_%s_utl' % (self.name, num))

                pointMtx = pmc.createNode('fourByFourMatrix', name='%s_pointOrientMtx_%s_utl' % (self.name, num))

                pointSideVec.outputX.connect(pointMtx.in00)
                pointSideVec.outputY.connect(pointMtx.in01)
                pointSideVec.outputZ.connect(pointMtx.in02)

                pointUpVec.outputX.connect(pointMtx.in10)
                pointUpVec.outputY.connect(pointMtx.in11)
                pointUpVec.outputZ.connect(pointMtx.in12)

                pointFrontVec.outputR.connect(pointMtx.in20)
                pointFrontVec.outputG.connect(pointMtx.in21)
                pointFrontVec.outputB.connect(pointMtx.in22)

                d.outputTranslateX.connect(pointMtx.in30)
                d.outputTranslateY.connect(pointMtx.in31)
                d.outputTranslateZ.connect(pointMtx.in32)

                parentMtx = None
                if a == 0:
                    parentMtx = coreUtils.inverseMatrix(mtx.output, name='%s_parentInverseMtx_%s_utl' % (self.name, num))
                else:
                    parentMtx = coreUtils.inverseMatrix(mtx.matrixSum, name='%s_parentInverseMtx_%s_utl' % (self.name, num))
                localOrientMtx = coreUtils.multiplyMatrices([pointMtx.output, parentMtx.outputMatrix], name='%s_pointMtx_%s_utl' % (self.name, num))

                yVec = coreUtils.matrixAxisToVector(localOrientMtx.matrixSum, name='%s_pointYVec_%s_utl' % (self.name, num), axis='y')
                scaleY = coreUtils.divide(1.0, yVec.outputY, name='%s_pointScaleY_%s_utl' % (self.name, num))

                xVec = coreUtils.matrixAxisToVector(localOrientMtx.matrixSum, name='%s_pointXVec_%s_utl' % (self.name, num), axis='x')
                scaleX = coreUtils.divide(1.0, xVec.outputX, name='%s_pointScaleX_%s_utl' % (self.name, num))

                d = coreUtils.decomposeMatrix(pointMtx.output, name='%s_pointMtxToSrt_%s_utl' % (self.name, num))
                d.outputTranslate.connect(loc.t)
                d.outputRotate.connect(loc.r)
                scaleY.outputX.connect(loc.sy)
                scaleX.outputX.connect(loc.sx)
