using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino;
using System;
using System.Collections.Generic;
using System.Linq;

namespace CustomOffset
{
    public class CustomOffsetComponent : GH_Component
    {
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
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "The Brep which faces need to be offset.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Offsets", "O", "Offsets as a list. Shorter list will repeat last number.", GH_ParamAccess.list);
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
            Brep brep = null;

            // Use the DA object to retrieve the data inside the first input parameter.
            if (!DA.GetData(0, ref brep)) { return; }
            if (!brep.IsValid) { return; }

            List<double> offsets = new List<double>();
            if (!DA.GetDataList(1, offsets)) { return; }
            if (offsets.Count == 0) { return; }

            Dictionary<BrepFace, double> facesOffsetDictionary = GetFacesOffsetDictionary(brep, offsets);

            Brep[] output = Offset(brep, facesOffsetDictionary, 0.1);

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
        //"A5FBC9D5-6198-44E7-8B12-42AADF320EE1");

        // main operation

        public Dictionary<BrepFace, double> GetFacesOffsetDictionary(Brep buildingBody, List<double> offsets)
        {
            Dictionary<BrepFace, double> d = new Dictionary<BrepFace, double>();


            buildingBody.Faces.ToList().ForEach(face => d.Add(face, offsets[face.FaceIndex]));
            return d;
        }

        public Brep[] Offset(Brep buildingBody, Dictionary<BrepFace, double> facesOffsetDictionary, double tolerance)
        {
            List<Brep> offsetFaces = new List<Brep>();

            // for each face the edges have a Vector3 associated representing the offset
            // based on the angle between the 2 faces
            Dictionary<int, Dictionary<BrepEdge, Vector3d>> faceEdgeVectors = new Dictionary<int, Dictionary<BrepEdge, Vector3d>>();

            // Populate for each edge for each face the offset Vector
            foreach (BrepFace face in facesOffsetDictionary.Keys)
            {
                int[] edgesIndex = face.AdjacentEdges();
                List<BrepEdge> surroundingEdges = buildingBody.Edges.Where(e => edgesIndex.Contains(e.EdgeIndex)).ToList();

                faceEdgeVectors[face.FaceIndex] = new Dictionary<BrepEdge, Vector3d>();

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

                    Vector3d scalingVector = line.PointAt(parameter) - face.GetBoundingBox(true).Center;

                    scalingVector.Unitize();
                    double angle = AngleRadCalculator(face, neighbourFace[0]) - 0.5 * Math.PI;

                    var scalingFactor =
                        facesOffsetDictionary[neighbourFace[0]] / Math.Cos(angle) -
                        facesOffsetDictionary[face] * Math.Tan(angle);

                    scalingVector *= scalingFactor;

                    faceEdgeVectors[face.FaceIndex][edge] = scalingVector;
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

                    List<BrepFace> commonFaces = buildingBody.Faces.Where(f => brepEdge.AdjacentFaces().Contains(f.FaceIndex)).ToList();
                    var commonFace = commonFaces.Where(f => f != face);

                    List<BrepFace> startFaces = buildingBody.Faces.Where(f => startAdjacentEdge[0].AdjacentFaces().Contains(f.FaceIndex)).ToList();
                    var startFace = startFaces.Where(f => f != face);
                    Point3d endOfStartAdjacentEdge = (startAdjacentEdge[0].PointAtStart.DistanceTo(brepEdge.PointAtStart) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance) ? startAdjacentEdge[0].PointAtEnd : startAdjacentEdge[0].PointAtStart;
                    Vector3d orientedVectorForStartCalculation = endOfStartAdjacentEdge - brepEdge.PointAtStart;
                    double startCorrectingAngle2 = Vector3d.VectorAngle(brepEdge.TangentAt(0.5), orientedVectorForStartCalculation) - 0.5 * Math.PI;

                    List<BrepFace> endFaces = buildingBody.Faces.Where(f => endAdjacentEdge[0].AdjacentFaces().Contains(f.FaceIndex)).ToList();
                    var endFace = endFaces.Where(f => f != face);
                    Point3d endOfEndAdjacentEdge = (endAdjacentEdge[0].PointAtStart.DistanceTo(brepEdge.PointAtEnd) < RhinoDoc.ActiveDoc.ModelAbsoluteTolerance) ? endAdjacentEdge[0].PointAtEnd : endAdjacentEdge[0].PointAtStart;
                    Vector3d orientedVectorForEndCalculation = endOfEndAdjacentEdge - brepEdge.PointAtEnd;
                    double endCorrectingAngle2 = Vector3d.VectorAngle(brepEdge.PointAtStart - brepEdge.PointAtEnd, orientedVectorForEndCalculation) - 0.5 * Math.PI;

                    Vector3d startVector = faceVectorMap.TryGetValue(startAdjacentEdge[0], out var vector0) ? vector0 : Vector3d.Unset;

                    double startAdditionalVector = faceVectorMap[brepEdge].Length * Math.Tan(startCorrectingAngle2);
                    double startAdditionalVector2 = faceVectorMap[brepEdge].Length * Math.Sin(startCorrectingAngle2);

                    double startScalingFactor =
                        (startVector.Length - startAdditionalVector2) / Math.Cos(startCorrectingAngle2);
                    startVector.Unitize();
                    startVector *= startScalingFactor;

                    Vector3d endVector = faceVectorMap.TryGetValue(endAdjacentEdge[0], out var vector1) ? vector1 : Vector3d.Unset;

                    double endAdditionalVector = faceVectorMap[brepEdge].Length * Math.Tan(endCorrectingAngle2);
                    double endAdditionalVector2 = faceVectorMap[brepEdge].Length * Math.Sin(endCorrectingAngle2);

                    double endScalingFactor =
                        (endVector.Length - endAdditionalVector2) / Math.Cos(endCorrectingAngle2);
                    endVector.Unitize();
                    endVector *= endScalingFactor;

                    Curve curve = brepEdge.DuplicateCurve();

                    curve.Transform(Transform.Scale(curve.PointAtStart, (curve.GetLength() + endVector.Length) / curve.GetLength()));
                    curve.Transform(Transform.Scale(curve.PointAtEnd, (curve.GetLength() + startVector.Length) / curve.GetLength()));

                    Vector3d offsetVector1 = face.NormalAt(0.5, 0.5);
                    offsetVector1.Unitize();
                    offsetVector1 *= facesOffsetDictionary[face];
                    curve.Translate(offsetVector1);

                    curve.Translate(faceVectorMap[brepEdge]);

                    edgesToBeStretched.Add(curve);
                }

                Brep reconstructedFace = Brep.CreatePlanarBreps(edgesToBeStretched, 0.1)[0];

                offsetFaces.Add(reconstructedFace);
            }

            Brep[] offsetCube = Brep.JoinBreps(offsetFaces, tolerance);

            return offsetCube;
        }

        public double AngleRadCalculator(BrepFace b1, BrepFace b2)
        {
            Vector3d v1, v2;
            v1 = b1.NormalAt(0.5, 0.5);
            v2 = b2.NormalAt(0.5, 0.5);

            double angleRad = Math.Abs(Vector3d.VectorAngle(v1, v2) - Math.PI);
            double angleDegrees = RhinoMath.ToDegrees(angleRad);

            return angleRad;
        }
    }
}