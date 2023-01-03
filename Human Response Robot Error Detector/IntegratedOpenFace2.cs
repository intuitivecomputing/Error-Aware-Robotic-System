using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Psi;
using Microsoft.Psi.Imaging;
using CppInterop.LandmarkDetector;
using FaceAnalyser_Interop;
using FaceDetectorInterop;
using OpenCVWrappers;


namespace HumanResponseRobotDetector
{
    public class IntegratedOpenFace2
    {
        private static FaceModelParameters faceModelParameters;
        private static FaceDetector faceDetector;
        private static CLNF landmarkDetector;
        private static FaceAnalyserManaged faceAnalyser;

        public IntegratedOpenFace2(Pipeline p)
        {
            In = p.CreateReceiver<Shared<Image>>(this, OpenFaceAUExtractor, nameof(In));
            Out = p.CreateEmitter<Dictionary<string, Tuple<double, double>>>(this, nameof(Out));
            Conf = p.CreateEmitter<double>(this, nameof(Conf));

            p.PipelineRun += initializeOpenFace;
            p.PipelineCompleted += OnPipelineCompleted;
        }

        public Receiver<Shared<Image>> In { get; private set; }

        // String is AU name, tuple is <intensity, occurance>
        public Emitter<Dictionary<string, Tuple<double, double>>> Out { get; private set; }

        public Emitter<double> Conf { get; private set; }

        // Initialize variables to set up OpenFace
        private void initializeOpenFace(object sender, PipelineRunEventArgs e)
        {
            faceModelParameters = new FaceModelParameters(AppDomain.CurrentDomain.BaseDirectory, true, false, false);
            faceModelParameters.optimiseForVideo();

            faceDetector = new FaceDetector(faceModelParameters.GetHaarLocation(), faceModelParameters.GetMTCNNLocation());
            if (!faceDetector.IsMTCNNLoaded())
            {
                faceModelParameters.SetFaceDetector(false, true, false);
            }

            landmarkDetector = new CLNF(faceModelParameters);
            faceAnalyser = new FaceAnalyserManaged(AppDomain.CurrentDomain.BaseDirectory, true, 112, true);

            landmarkDetector.Reset();
            faceAnalyser.Reset();
        }

        private void OpenFaceAUExtractor(Shared<Image> inputImage, Envelope envelope)
        {
            double confidence = 0.0;
            Dictionary<string, Tuple<double, double>> actionUnits = new Dictionary<string, Tuple<double, double>>();

            using (var colorSharedImage = ImagePool.GetOrCreate(inputImage.Resource.Width, inputImage.Resource.Height, inputImage.Resource.PixelFormat))
            {
                inputImage.Resource.CopyTo(colorSharedImage.Resource);
                var colorImage = new RawImage(colorSharedImage.Resource.ToBitmap());
                if (landmarkDetector.DetectLandmarksInVideo(colorImage, faceModelParameters))
                {
                    var landmarks = landmarkDetector.CalculateAllLandmarks();
                    confidence = landmarkDetector.GetConfidence();
                    var (actionUnitIntensities, actionUnitOccurences) = faceAnalyser.PredictStaticAUsAndComputeFeatures(colorImage, landmarks);
                    actionUnits = actionUnitIntensities.ToDictionary(kv => kv.Key, kv => new Tuple<double, double>(kv.Value, actionUnitOccurences[kv.Key]));
                }

                Conf.Post(confidence, envelope.OriginatingTime);
                Out.Post(actionUnits, envelope.OriginatingTime);
            }
        }

        private void OnPipelineCompleted(object sender, PipelineCompletedEventArgs e)
        {
        }
    }
}
