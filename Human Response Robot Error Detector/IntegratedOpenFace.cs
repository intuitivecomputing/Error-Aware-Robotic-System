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
    // Calculates the AUs per timestep given a video input and outputs the facial detection confidence
    // and the AUs calculated.

    public class IntegratedOpenFace
    {
        private static FaceModelParameters faceModelParameters;
        private static FaceDetector faceDetector;
        private static CLNF landmarkDetector;
        private static FaceAnalyserManaged faceAnalyser;

        public IntegratedOpenFace(Pipeline p)
        {
            // Input into the component is a frame from the video
            In = p.CreateReceiver<Shared<Image>>(this, OpenFaceAUExtractor, nameof(In));
            // Outputs from the component is a dictionary of the AUs' occurence and intensity and facial detection confidence
            Out = p.CreateEmitter<Dictionary<string, Tuple<double, double>>>(this, nameof(Out));
            Conf = p.CreateEmitter<double>(this, nameof(Conf));

            // Add this component to the pipeline
            p.PipelineRun += initializeOpenFace;
            p.PipelineCompleted += OnPipelineCompleted;
        }

        public Receiver<Shared<Image>> In { get; private set; }

        // String is AU name, tuple is <intensity, occurance>
        public Emitter<Dictionary<string, Tuple<double, double>>> Out { get; private set; }

        // Facial detection confidence
        public Emitter<double> Conf { get; private set; }

        // Initialize variables to set up OpenFace for AU
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

        // This function calculates the AUs and facial detection confidence for a given image. The arguments are:
        // inputImage: image collected from camera for the purpose of AU detection
        // envelope: contains information such as originating time
        private void OpenFaceAUExtractor(Shared<Image> inputImage, Envelope envelope)
        {
            // Create empty double to store facial detection confidence
            double confidence = 0.0;
            // Create an empty dictionary to store AU occurence and intensities
            Dictionary<string, Tuple<double, double>> actionUnits = new Dictionary<string, Tuple<double, double>>();

            // Converts image input into a raw image that can used to detect facial landmarks
            using (var colorSharedImage = ImagePool.GetOrCreate(inputImage.Resource.Width, inputImage.Resource.Height, inputImage.Resource.PixelFormat))
            {
                inputImage.Resource.CopyTo(colorSharedImage.Resource);
                var colorImage = new RawImage(colorSharedImage.Resource.ToBitmap());
                // Checks if landmark detection was successful
                if (landmarkDetector.DetectLandmarksInVideo(colorImage, faceModelParameters))
                {
                    var landmarks = landmarkDetector.CalculateAllLandmarks();
                    // Calculate confidence for landmark detection
                    confidence = landmarkDetector.GetConfidence();
                    //Calculate the AUs
                    var (actionUnitIntensities, actionUnitOccurences) = faceAnalyser.PredictStaticAUsAndComputeFeatures(colorImage, landmarks);
                    actionUnits = actionUnitIntensities.ToDictionary(kv => kv.Key, kv => new Tuple<double, double>(kv.Value, actionUnitOccurences[kv.Key]));
                }

                // Output the facial detection confidence and AUs
                Conf.Post(confidence, envelope.OriginatingTime);
                Out.Post(actionUnits, envelope.OriginatingTime);
            }
        }

        private void OnPipelineCompleted(object sender, PipelineCompletedEventArgs e)
        {
        }
    }
}
