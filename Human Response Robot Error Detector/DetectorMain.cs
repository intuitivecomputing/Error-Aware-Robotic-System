using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Psi;
using Microsoft.Psi.Imaging;
using Microsoft.Psi.Media;
using Microsoft.Psi.AzureKinect;
using Microsoft.Azure.Kinect.Sensor;
using Microsoft.Psi.Audio;
using Microsoft.Psi.Speech;
using Microsoft.Psi.Interop.Format;
using Microsoft.Psi.Interop.Transport;
using System.IO;

namespace HumanResponseRobotDetector
{
    class DetectorMain
    {
        // Indicator of whether the robot is moving
        public static double isMoving = 0;
        // Indicator of whether to use of social signal modeling error detection (true for yes)
        public static bool isActiveDetection = false;
        // Indicator of whether potential error detected and robot is querying
        public static bool isQuery = false;
        // Indicator of whether recovery is happened
        public static bool isRecovering = false;
        // Stores how many verbal commands have been stated
        public static double commandCount = 0;


        static void Main(string[] args)
        {
            using (var p = Pipeline.Create(enableDiagnostics: true))
            {
                //Specify File Names and Address
                var storeName = "trial";
                var storeAddress = "XXXXXXXXX";
                var outputAUName = "XXXXXXXXX\\OpenFaceOutput.csv";
                var mlOutputName = "XXXXXXXXX\\MLOutput.csv";

                // Create the store
                var store = PsiStore.Create(p, storeName, storeAddress);

                // Initialize bridging to machine running the robot to command robot
                // Linux IP
                var commandWriter = new NetMQWriter<string>(p, "commands", "tcp://XXX.XXX.X.XX:X", MessagePackFormat.Instance);
                // Initialize bridging to python for social signal ML
                // Local IP
                var AUWriter = new NetMQWriter<double[]>(p, "AUs Intensities", "tcp://XXX.X.X.X:X", MessagePackFormat.Instance);

                // Initialize bridging back from machine running the robot to psi
                var robotDoneSource = new NetMQSource<bool>(p, "isDone", "tcp://XXX.XXX.X.XX:X", MessagePackFormat.Instance);
                // Initialize bridging back from social signal ML to psi
                // Local IP
                var errorTimestepSource = new NetMQSource<string>(p, "isNewError", "tcp://XXX.X.X.X:X", MessagePackFormat.Instance);

                // Kinect Camera 1
                var cam = new AzureKinectSensor(p, new AzureKinectSensorConfiguration()
                {
                    DeviceIndex = 0,
                    OutputColor = true,
                    CameraFPS = FPS.FPS30,
                    //WiredSyncMode = WiredSyncMode.Master,
                    PowerlineFrequency = AzureKinectSensorConfiguration.PowerlineFrequencyTypes.SixtyHz,
                    ExposureTime = TimeSpan.FromTicks(80000),
                    ColorResolution = ColorResolution.R720p
                });
                var imageStream = cam.ColorImage.EncodeJpeg(75, DeliveryPolicy.LatestMessage);
                imageStream.Write("Image1", store);
                var decodedImageStream = imageStream.Decode();

                // Kinect Camera 2 
                var cam2 = new AzureKinectSensor(p, new AzureKinectSensorConfiguration()
                {
                    DeviceIndex = 1,
                    OutputColor = true,
                    //WiredSyncMode = WiredSyncMode.Subordinate,
                    CameraFPS = FPS.FPS30,
                    PowerlineFrequency = AzureKinectSensorConfiguration.PowerlineFrequencyTypes.SixtyHz,
                    ExposureTime = TimeSpan.FromTicks(80000),
                    ColorResolution = ColorResolution.R720p
                });
                var imageStream2 = cam2.ColorImage.EncodeJpeg(75, DeliveryPolicy.LatestMessage);
                imageStream2.Write("Image2", store);
                var decodedImageStream2 = imageStream2.Decode();

                //Audio 
                var audioSource = new AudioCapture(p, new AudioCaptureConfiguration()
                {
                    DeviceName = "Microphone (USB PnP Audio Device)",
                    Format = WaveFormat.Create16kHz1Channel16BitPcm()
                });

                // Sync audio with video so start at same time
                var camWithAudio = audioSource.Join(decodedImageStream, RelativeTimeInterval.Past());

                // Extract audio from the combined video audio stream
                var justAudio = camWithAudio.Select(m => { var (aud, img) = m; return aud; });
                justAudio.Write("Audio", store);

                // Create robot moving stream so can sync ml to it
                var tempStream = Generators.Repeat(p, 0, TimeSpan.FromTicks(80000)).Select(m => { return isMoving; });
                var robotVideoSyncedStream = tempStream.Join(camWithAudio, RelativeTimeInterval.Past());
                var robotMovingStream = robotVideoSyncedStream.Select(m => { var (move, aud, img) = m; return move; });
                robotMovingStream.Write("robotMoving", store);

                // Send imagestream from each camera to the openface component to get the AUs and confidence
                var openFaceComponent = new IntegratedOpenFace(p);
                var openFaceComponent2 = new IntegratedOpenFace2(p);
                // Every 10th image is sent to be processed making each timestep 1/3 of a second
                decodedImageStream.Where((img, e) => e.SequenceId % 10 == 0).PipeTo(openFaceComponent.In);
                decodedImageStream2.Where((img, e) => e.SequenceId % 10 == 0).PipeTo(openFaceComponent2.In);

                // Output face detection confidence from each openface component
                var confidence = openFaceComponent.Conf;
                confidence.Write("Confidence", store);
                var confidence2 = openFaceComponent2.Conf;
                confidence2.Write("Confidence2", store);
                // Output AU detection from each openface compenent
                var AUs = openFaceComponent.Out;
                var AUs2 = openFaceComponent2.Out;

                // Combine all of the streams confidences and AUs to some time base 
                var joinFace = confidence.Join(confidence2, RelativeTimeInterval.Past()).Join(AUs, RelativeTimeInterval.Past()).Join(AUs2, RelativeTimeInterval.Past()).Join(robotMovingStream, RelativeTimeInterval.Past());
                // Write to CSV file to store the AUs calculated
                var csvLines = new List<string>();
                // Check if the csv file exists and if doesn't create a new line with all of the headers 
                if (!File.Exists(outputAUName))
                {
                    File.WriteAllLines(outputAUName, new List<string> {"Originating Time,Confidence,AU04_i,AU04_o,AU06_i,AU06_o,AU07_i,AU07_o,AU10_i,AU10_o,AU12_i,AU12_o," +
                        "AU14_i,AU14_o,AU01_i,AU01_o,AU02_i,AU02_o,AU05_i,AU05_o,AU09_i,AU09_o,AU15_i,AU15_o,AU17_i,AU17_o,AU20_i,AU20_o,AU23_i,AU23_o,AU25_i,AU25_o," +
                        "AU26_i,AU26_o,AU45_i,AU45_o"});
                }
                // Selected AUs for each timestep from one of the cameras depending on which one has the more confident facial detection and then
                // writes the AUs to the csv file.
                // If both facial detection is below 50\%, then it writes a string of zeros for the AUs
                var AUStream = joinFace.Select((m, e) =>
                {
                    var (c1, c2, au1, au2, mov) = m;
                    var temp = new List<double>(new double[17]);
                    temp.Insert(0, mov);
                    var vals = temp.ToArray();
                    // Checks if the facial detection confidence is both below 50%
                    if (c1 < 0.5 && c2 < 0.5)
                    {
                        var all = new List<double>(new double[35]);
                        csvLines.Add(e.OriginatingTime + "," + all.ToArray().Select(a => a.ToString()).Aggregate((i, j) => i + "," + j));
                        File.AppendAllLines(outputAUName, csvLines);
                        csvLines = new List<string>();
                    }
                    else if (c1 > c2)
                    {
                        csvLines.Add(e.OriginatingTime + "," + c1 + $",{au1.Values.ToArray().Select(a => a.Item1 + "," + a.Item2).Aggregate((i, j) => i + "," + j)}");
                        File.AppendAllLines(outputAUName, csvLines);
                        csvLines = new List<string>();

                        vals = au1.Values.ToArray().Select(a => a.Item1).ToArray();
                        var t = vals.ToList();
                        t.Insert(0, mov);
                        vals = t.ToArray();
                    }
                    else
                    {
                        csvLines.Add(e.OriginatingTime + "," + c2 + $",{au2.Values.ToArray().Select(a => a.Item1 + "," + a.Item2).Aggregate((i, j) => i + "," + j)}");
                        File.AppendAllLines(outputAUName, csvLines);
                        csvLines = new List<string>();

                        vals = au2.Values.ToArray().Select(a => a.Item1).ToArray();
                        var t = vals.ToList();
                        t.Insert(0, mov);
                        vals = t.ToArray();
                    }
                    return vals;
                });
                // Pipe the AUs to the social signal ML algorithm for error detection
                AUStream.PipeTo(AUWriter);

                // Output from the ML algorithm
                var errorStringStream = errorTimestepSource.Out;
                var errorTimestepStream = errorStringStream.Select(m =>
                {
                    return Array.ConvertAll(m.Split(','), Double.Parse);
                });
                // Extract whether the current timestep was classfied as an error timestep
                errorTimestepStream.Select(m => { return m[1]; }).Write("errorTimestep", store);
                // Extract whether the current timestep was also when an new error was detected
                var errorStopStream = errorTimestepStream.Select(m => { return m[3]; });
                errorStopStream.Write("errorPotentStop", store);

                // If a new error is indicated and the domain-specific information allows it, the
                // system will send to the robot controller that a possible error has happened
                var tempError = errorStopStream.Select(m =>
                {
                    var cmmd = "";
                    // Check if system is already waiting for a response to an error detected query, has turned
                    // on the social signal input detection, is already needing to recover from a previous error
                    // detection, has a new error detected, and is above command count of 8 (remove novelty effect)
                    if (!isQuery && isActiveDetection && !isRecovering && m == 1 && commandCount >= 9)
                    {
                        // Indicates that the system needs to query the user 
                        isQuery = true;
                        cmmd = "possible";
                    }
                    return cmmd;
                });
                // Create streamm indicating potential errors as detected by ml
                var potentError = tempError.Where(m => m != "");

                // Write to CSV ML file
                csvLines = new List<string>();
                // Check if the csv file exists and if doesn't create a new line with all of the headers 
                if (!File.Exists(mlOutputName))
                {
                    File.WriteAllLines(mlOutputName, new List<string> { "Originating Time,robotMoving,isErrorTimeStep,errorTimeStepConfidence,errorStop" });
                }
                errorTimestepStream.Do((m, e) => 
                {
                    csvLines.Add(e.OriginatingTime + "," + m.ToArray().Select(item => item.ToString()).Aggregate((i, j) => i + "," + j));
                    File.AppendAllLines(mlOutputName, csvLines);
                    csvLines = new List<string>();
                });

                // Initialize speech recognizer
                var recognizer = new SystemSpeechRecognizer(p, new SystemSpeechRecognizerConfiguration()
                {
                    Language = "en-US",
                    // Define file with all of the valid commands 
                    Grammars = new GrammarInfo[]
                        {
                                new GrammarInfo() { Name = "Command", FileName = "Commands.grxml" },
                        },
                });
                // Pipe audio to the speech recognizer
                justAudio.PipeTo(recognizer);

                // Command from speech recognizer
                var commands = recognizer.Out.Where(result => result.IsFinal);
                // Print final recognition results
                commands.Do(result =>
                {
                    Console.WriteLine($"{result.Text} (confidence: {result.Confidence})");
                });

                // Extract specific item requested,  explicit error reporting, and response to queries
                var neededObject = commands.Select(result =>
                {
                    var cmmd = "";
                    var conf = result.Confidence;
                    // Only add as a command when confidence is greater than 0.85, the robot is not moving, 
                    // and not asking about how the robot is doing
                    if (conf >= 0.85 && !isQuery)
                    {
                        // Check if the command is requesting a pipe and is not already moving 
                        if (result.Text.Split(' ').Last() == "pipe" && isMoving == 0)
                        {
                            commandCount = commandCount + 1;
                            var temp = result.Text.Split(' ');
                            // Extract the requested pipe's color
                            cmmd = temp[temp.Length - 2];
                            // Indicate that the robot is moving
                            isMoving = 1;
                            return cmmd;
                        }
                        // Manually reporting an error can happen either when robot is or isnt' moving but not
                        // when the robot needs to recover
                        else if (result.Text.Split(' ').Last() == "error" && !isRecovering)
                        {
                            cmmd = "error";
                            // Indicate that the robot needs to recover from error
                            isRecovering = true;
                            // Indicate that the robot is moving
                            isMoving = 1;
                            return cmmd;
                        }
                    }
                    // Check if the voice command is in response to a query. It is the case when the confidence is
                    // above or equal to 0.85, there was query sent, and social signal ml detection was turned on.
                    // The query, in this case, is "Am I doing alright?"
                    else if (conf >= 0.85 && isQuery && isActiveDetection)
                    {
                        // Check if negative response, so an error occurred
                        if (result.Text == "No you aren't")
                        {
                            cmmd = "error";
                            Console.WriteLine("Automatic Error Detected");
                            // Indicate query has been answered and no longer needed
                            isQuery = false;
                            // Indicate that the robot needs to recover from error
                            isRecovering = true;
                            return cmmd;
                        }
                        // Check if affirmative response, so no error occurred
                        else if (result.Text == "Yes you are")
                        {
                            cmmd = "resume";
                            Console.WriteLine("False Positive");
                            // Indicate query has been answered and no longer needed
                            isQuery = false;
                            isMoving = 0;
                            return cmmd;
                        }
                    }
                    return "";
                });

                // Merge verbal commands with the potential error detected indication to send to the robot controller 
                var neededCommand = neededObject.Where(m => m != "").Merge(potentError).Select(m => { Console.WriteLine(m.Data); Console.WriteLine(m.CreationTime); return m.Data; });
                // Sychronize all commands with the video so on same time scale
                var neededCommands = neededCommand.Join(robotVideoSyncedStream, RelativeTimeInterval.Past()).Select(m => { var (cmmd, move, aud, img) = m; return cmmd; });
                // Send all of the commands to the robot controller
                neededCommands.PipeTo(commandWriter);

                // Output from robot controller of whether the robot is done moving in response to commands sent
                robotDoneSource.Do((m, e) =>
                {
                    // Check if done moving
                    if (m == true)
                    {
                        isMoving = 0;
                        // Also if done moving and needed to recover then the recovery is done
                        if (isRecovering)
                        {
                            isRecovering = false;
                        }
                    }
                });

                // Write the diagnostics stream to the store
                p.Diagnostics.Write("Diagnostics", store);

                // Run the pipeline
                p.RunAsync();

                Console.WriteLine("Press any key to finish recording");
                Console.ReadKey();
            }
        }
    }
}
