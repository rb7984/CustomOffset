using Grasshopper.GUI.SettingsControls;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry.Delaunay;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;

namespace CustomOffset
{
    public class CustomOffsetComponent : GH_Component
    {
        // fields
        private List<Point3d> pointsForDisplay;
        private List<int> facesIndexesForDisplay;
        private bool switchVisualise;

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public CustomOffsetComponent()
          : base("Custom Offset", "CO",
            "Offset multiple planar Surfaces given multiple fixed amounts",
            "Surface", "Util")
        {
            pointsForDisplay = new List<Point3d>();
            facesIndexesForDisplay = new List<int>();
            switchVisualise = false;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "The Brep which faces need to be offset.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Offsets", "O", "Offsets as a list. Shorter list will repeat last number.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Visualise", "V", "A switch for face index visualisation.", GH_ParamAccess.item);
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Out", "O", "Output Brep", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            DA.GetData(2, ref switchVisualise);

            // Reset the display Lists
            pointsForDisplay.Clear();
            facesIndexesForDisplay.Clear();

            Brep brep = null;

            // Use the DA object to retrieve the data inside the first input parameter.
            if (!DA.GetData(0, ref brep)) { return; }
            if (!brep.IsValid) { return; }

            List<double> offsets = new List<double>();
            if (!DA.GetDataList(1, offsets)) { return; }
            if (offsets.Count == 0) { return; }

            Dictionary<BrepFace, double> facesOffsetDictionary = GetFacesOffsetDictionary(brep, offsets);

            Brep[] output = Offset(brep, facesOffsetDictionary, 0.1);

            if (output == null)
            {
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Some of the offset Faces do not have a closed Envelope.");
                return;
            }
            else
                DA.SetData(0, output[0]);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resource1.CO;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("dc359d87-d785-4345-8ebc-e2738494ae73");

        #region Main Operation
        public Dictionary<BrepFace, double> GetFacesOffsetDictionary(Brep buildingBody, List<double> offsets)
        {
            Dictionary<BrepFace, double> d = new Dictionary<BrepFace, double>();

            if (offsets.Count < buildingBody.Faces.Count)
            {
                offsets.AddRange(Enumerable.Repeat(offsets[offsets.Count - 1], buildingBody.Faces.Count - offsets.Count).ToList());
            }

            buildingBody.Faces.ToList().ForEach(face => d.Add(face, offsets[face.FaceIndex]));
            return d;
        }

        public Brep[] Offset(Brep buildingBody, Dictionary<BrepFace, double> facesOffsetDictionary, double tolerance)
        {
            List<Brep> offsetFaces = new List<Brep>();

            // for each face the edges have a Vector3d associated representing the offset
            // based on the angle between the 2 faces
            Dictionary<int, Dictionary<BrepEdge, Vector3d>> faceEdgeVectors = new Dictionary<int, Dictionary<BrepEdge, Vector3d>>();
            Dictionary<int, Dictionary<BrepEdge, double>> faceEdgeVectors2 = new Dictionary<int, Dictionary<BrepEdge, double>>();
            Dictionary<int, Dictionary<int, double>> faceVertexAngles = CalculatePlanarAngle(buildingBody);

            // Populate for each edge for each face the offset Vector
            foreach (BrepFace face in facesOffsetDictionary.Keys)
            {
                pointsForDisplay.Add(face.GetBoundingBox(false).Center);
                facesIndexesForDisplay.Add(face.FaceIndex);

                int[] edgesIndex = face.AdjacentEdges();
                List<BrepEdge> surroundingEdges = buildingBody.Edges.Where(e => edgesIndex.Contains(e.EdgeIndex)).ToList();

                faceEdgeVectors[face.FaceIndex] = new Dictionary<BrepEdge, Vector3d>();
                faceEdgeVectors2[face.FaceIndex] = new Dictionary<BrepEdge, double>();

                foreach (BrepEdge edge in surroundingEdges)
                {
                    Curve edgeCurve = edge.DuplicateCurve();
                    edgeCurve.Domain = new Interval(0, 1);

                    int[] neighbourFacesByFace = face.AdjacentFaces();
                    int[] neighbourFacesByEdge = edge.AdjacentFaces();

                    int[] neighbourFaces = neighbourFacesByEdge.Intersect(neighbourFacesByFace).ToArray();

                    List<BrepFace> neighbourFace = buildingBody.Faces.Where(f => neighbourFaces.Contains(f.FaceIndex)).ToList();

                    Curve line = edge.ToNurbsCurve();
                    double parameter;
                    line.ClosestPoint(face.GetBoundingBox(true).Center, out parameter);

                    Vector3d scalingVector;
                    double angle = CalculateDihedralAngle(face, edge, neighbourFace[0], out scalingVector) - 0.5 * Math.PI;

                    var scalingFactor =
                        facesOffsetDictionary[neighbourFace[0]] / Math.Cos(angle) -
                        facesOffsetDictionary[face] * Math.Tan(angle);

                    scalingVector *= scalingFactor;

                    faceEdgeVectors[face.FaceIndex][edge] = scalingVector;
                    faceEdgeVectors2[face.FaceIndex][edge] = scalingFactor;
                }
            }

            // Scaling the edge base on the dictionary
            foreach (BrepFace face in facesOffsetDictionary.Keys)
            {
                int[] edgesIndex = face.AdjacentEdges();
                List<BrepEdge> surroundingEdges = buildingBody.Edges.Where(e => edgesIndex.Contains(e.EdgeIndex)).ToList();

                List<Curve> edgesToBeStretched = new List<Curve>(edgesIndex.Length);

                var faceVectorMap = new Dictionary<BrepEdge, Vector3d>();
                faceEdgeVectors.TryGetValue(face.FaceIndex, out faceVectorMap);

                var faceVectorMap2 = new Dictionary<BrepEdge, double>();
                faceEdgeVectors2.TryGetValue(face.FaceIndex, out faceVectorMap2);

                foreach (BrepEdge brepEdge in surroundingEdges)
                {
                    List<BrepEdge> startAdjacentEdge = surroundingEdges.Where(e =>
                    e != brepEdge &&
                    (
                        brepEdge.PointAtStart.DistanceTo(e.PointAtStart) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance ||
                        brepEdge.PointAtStart.DistanceTo(e.PointAtEnd) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
                    )
                        ).ToList();

                    List<BrepEdge> endAdjacentEdge = surroundingEdges.Where(e =>
                    e != brepEdge &&
                    (
                        brepEdge.PointAtEnd.DistanceTo(e.PointAtStart) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance ||
                        brepEdge.PointAtEnd.DistanceTo(e.PointAtEnd) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
                    )
                        ).ToList();

                    Point3d endOfStartAdjacentEdge = (startAdjacentEdge[0].PointAtStart.DistanceTo(brepEdge.PointAtStart) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance) ? startAdjacentEdge[0].PointAtEnd : startAdjacentEdge[0].PointAtStart;
                    Vector3d orientedVectorForStartCalculation = endOfStartAdjacentEdge - brepEdge.PointAtStart;
                    double startCorrectingAngle2 = faceVertexAngles[face.FaceIndex][brepEdge.StartVertex.VertexIndex] - .5 * Math.PI;

                    Point3d endOfEndAdjacentEdge = (endAdjacentEdge[0].PointAtStart.DistanceTo(brepEdge.PointAtEnd) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance) ? endAdjacentEdge[0].PointAtEnd : endAdjacentEdge[0].PointAtStart;
                    Vector3d orientedVectorForEndCalculation = endOfEndAdjacentEdge - brepEdge.PointAtEnd;
                    double endCorrectingAngle2 = faceVertexAngles[face.FaceIndex][brepEdge.EndVertex.VertexIndex] - 0.5 * Math.PI;

                    double startVector2 = faceVectorMap2.TryGetValue(startAdjacentEdge[0], out var double0) ? double0 : 0;
                    double startAdditionalVector2 = faceVectorMap2[brepEdge] * Math.Sin(startCorrectingAngle2);
                    double startScalingFactor2 = (startVector2 - startAdditionalVector2) / Math.Cos(startCorrectingAngle2);

                    double endVector2 = faceVectorMap2.TryGetValue(endAdjacentEdge[0], out var double1) ? double1 : 0;
                    double endAdditionalVector2 = faceVectorMap2[brepEdge] * Math.Sin(endCorrectingAngle2);
                    double endScalingFactor2 = (endVector2 - endAdditionalVector2) / Math.Cos(endCorrectingAngle2);

                    Curve curve = brepEdge.DuplicateCurve();

                    if (!curve.Transform(Transform.Scale(curve.PointAtStart, (curve.GetLength() + endScalingFactor2) / curve.GetLength())))
                        return null;
                    if (!curve.Transform(Transform.Scale(curve.PointAtEnd, (curve.GetLength() + startScalingFactor2) / curve.GetLength())))
                        return null;

                    Vector3d offsetVector1 = face.NormalAt(0.5, 0.5);
                    offsetVector1.Unitize();
                    offsetVector1 *= facesOffsetDictionary[face];

                    curve.Translate(offsetVector1);
                    curve.Translate(faceVectorMap[brepEdge]);

                    edgesToBeStretched.Add(curve);
                }

                Brep[] breps = Brep.CreatePlanarBreps(edgesToBeStretched, 0.1);
                if (breps == null || breps.Length == 0)
                {
                    RhinoApp.WriteLine("Failed to create a planar Brep.");
                    return null;
                }

                Brep reconstructedFace = breps[0];

                offsetFaces.Add(reconstructedFace);
            }

            Brep[] offsetCube = Brep.JoinBreps(offsetFaces, tolerance);

            return offsetCube;
        }

        public double CalculateDihedralAngle(BrepFace face, BrepEdge edge, BrepFace adjacentFace, out Vector3d scalingVector)
        {
            Vector3d faceNormal = face.NormalAt(0.5, 0.5);
            Vector3d adjacentFaceNormal = adjacentFace.NormalAt(0.5, 0.5);

            var loop = face.OuterLoop;
            Curve loopasacurve = loop.To3dCurve();
            Polyline loopasapolyline = new Polyline();

            if (loopasacurve.IsPolyline())
            {
                //TODO maximum length può causare dei danni?
                loopasapolyline = loopasacurve.ToPolyline(0.1, 0.1, 0.1, 3000).ToPolyline();
            }

            loopasapolyline.MergeColinearSegments(0.1, true);
            Line[] ll = loopasapolyline.GetSegments();
            Line line = new Line();

            double distance = 100;
            int counter = 0;
            while (distance > 1)
            {
                double t;
                if (edge.ClosestPoint(ll[counter].PointAt(0.5), out t))
                {
                    distance = edge.PointAt(t).DistanceTo(ll[counter].PointAt(0.5));
                    line = ll[counter];
                    counter++;
                }
            }

            Vector3d testEdgeVector = line.Direction;
            testEdgeVector.Unitize();

            Vector3d rotatedFaceNormal = new Vector3d(faceNormal);
            rotatedFaceNormal.Rotate(Math.PI / 2, testEdgeVector);

            double rotatedDotProduct = Vector3d.Multiply(rotatedFaceNormal, adjacentFaceNormal);

            faceNormal.Unitize();
            adjacentFaceNormal.Unitize();

            double angle = Vector3d.VectorAngle(faceNormal, adjacentFaceNormal);

            if (rotatedDotProduct > 0)
            {
                angle = Math.PI - angle;
            }
            else
            {
                angle = Math.PI + angle;
            }

            scalingVector = Vector3d.CrossProduct(testEdgeVector, faceNormal);
            scalingVector.Unitize();

            return angle;
        }

        public Dictionary<int, Dictionary<int, double>> CalculatePlanarAngle(Brep body)
        {
            Dictionary<int, Dictionary<int, double>> faceVertexAngles = new Dictionary<int, Dictionary<int, double>>();

            if (body == null || body.Faces.Count == 0)
            {
                return null;
            }

            foreach (BrepFace face in body.Faces)
                if (!face.IsPlanar())
                {
                    return null;
                }

            foreach (BrepFace face in body.Faces)
            {
                faceVertexAngles[face.FaceIndex] = new Dictionary<int, double>();

                Vector3d faceNormal = face.NormalAt(0.5, 0.5);

                var loop = face.OuterLoop;
                Curve loopasacurve = loop.To3dCurve();
                Polyline loopasapolyline = new Polyline();

                if (loopasacurve.IsPolyline())
                {
                    loopasapolyline = loopasacurve.ToPolyline(0.1, 0.1, 0.1, loopasacurve.GetLength()).ToPolyline();
                }

                loopasapolyline.MergeColinearSegments(0.1, true);
                Line[] ll = loopasapolyline.GetSegments();

                int[] faceAdjacentEdgesIndexes = face.AdjacentEdges();
                var faceAdjacentEdges = body.Edges.Where(e => faceAdjacentEdgesIndexes.Contains(e.EdgeIndex));

                Dictionary<BrepEdge, Vector3d> edgeDirectionDictionary = new Dictionary<BrepEdge, Vector3d>();

                foreach (BrepEdge edge in faceAdjacentEdges)
                    foreach (Line l in ll)
                    {
                        Line line = new Line();

                        double distance = 100;
                        int maxAttempts = ll.Length;
                        int counter = 0;

                        while (distance > 1 && counter < maxAttempts)
                        {
                            double t;
                            if (edge.ClosestPoint(ll[counter].PointAt(0.5), out t))
                            {
                                distance = edge.PointAt(t).DistanceTo(ll[counter].PointAt(0.5));
                                line = ll[counter];
                                counter++;
                            }
                        }

                        Vector3d testEdgeVector = line.Direction;
                        testEdgeVector.Unitize();

                        edgeDirectionDictionary[edge] = testEdgeVector;
                    }

                foreach (BrepEdge edge in faceAdjacentEdges)
                {
                    BrepVertex bv = edge.StartVertex;
                    Vector3d testVector = edge.PointAtEnd - edge.PointAtStart;

                    if (testVector.IsParallelTo(edgeDirectionDictionary[edge]) > 0)
                        bv = edge.EndVertex;

                    int[] vertexAdjacentEdgesIndexes = bv.EdgeIndices();
                    var vertexAdjacentEdges = body.Edges.Where(e => vertexAdjacentEdgesIndexes.Contains(e.EdgeIndex));

                    var nextBrepEdges = vertexAdjacentEdges.Where(e => e != edge && edgeDirectionDictionary.ContainsKey(e)).ToList();
                    if (nextBrepEdges.Count == 0)
                    {
                        continue;
                    }
                    var nextBrepEdge = nextBrepEdges[0];

                    Vector3d va = -edgeDirectionDictionary[edge];
                    Vector3d vb = edgeDirectionDictionary[nextBrepEdge];

                    double angle = Vector3d.VectorAngle(va, vb);
                    Vector3d cp = Vector3d.CrossProduct(va, vb);

                    if (cp.IsParallelTo(faceNormal) == 1)
                        angle = (Math.PI * 2) - angle;

                    faceVertexAngles[face.FaceIndex][bv.VertexIndex] = angle;
                }
            }

            return faceVertexAngles;
        }

        #endregion

        #region Visualisation
        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            if (switchVisualise)
                for (int i = 0; i < pointsForDisplay.Count; i++)
                {
                    Plane plane;
                    args.Viewport.GetCameraFrame(out plane);
                    plane.Origin = pointsForDisplay[i];

                    double pixelsPerUnit;
                    args.Viewport.GetWorldToScreenScale(pointsForDisplay[i], out pixelsPerUnit);
                    args.Display.Draw3dText(facesIndexesForDisplay[i].ToString(), System.Drawing.Color.Black, plane, 25 / pixelsPerUnit, "Lucida Console", false, false, TextHorizontalAlignment.Center, TextVerticalAlignment.Middle);
                }

            //base.DrawViewportMeshes(args);
        }

        #endregion
    }
}