using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.IO;

namespace HandImageExtractor
{
    class HandImageExtractor
    {
        private int CROP_WINDOW_WIDTH = 150; // in pixels
        private int CROP_WINDOW_HEIGHT = 150; // in pixels

        private KinectSensor camDevice;
        private Skeleton activeSkeleton;
        private WriteableBitmap colorBitmap;
        private byte[] colorPixels;
        private int counter = 0;

        /// <summary>
        /// 
        /// No argument constructor.
        /// 
        /// </summary>
        public HandImageExtractor()
        {
            // Find Kinect Sensor
            camDevice = KinectSensor.KinectSensors[0]; 
            
            // Enable Data Streaming
            EnableDataStreaming();

            // Start Kinect Device
            StartCameraDevice();
        }

        /// <summary>
        /// 
        /// Constructor.
        /// 
        /// </summary>
        /// <param name="cropWindowWidth">Crop frame width dimension in pixels</param>
        /// <param name="cropWindowHeight">Crop frame height dimension in pixels</param>
        public HandImageExtractor(int cropWindowWidth, int cropWindowHeight)
        {
            // Find Kinect Sensor
            camDevice = KinectSensor.KinectSensors[0];

            // Enable Data Streaming
            EnableDataStreaming();

            // Start Kinect Device
            StartCameraDevice();

            // set crop frame dimensions
            this.CROP_WINDOW_WIDTH = cropWindowWidth;
            this.CROP_WINDOW_HEIGHT = cropWindowHeight;
        }

        /// <summary>
        /// 
        /// Enables Skeleton, Color and Depth Data Streaming
        /// 
        /// </summary>
        private void EnableDataStreaming()
        {
            if (this.camDevice != null)
            {
                this.camDevice.SkeletonStream.Enable();
                this.camDevice.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                this.camDevice.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
               // this.camDevice.ColorStream.Enable(ColorImageFormat.InfraredResolution640x480Fps30);
            }
        }

        /// <summary>
        /// 
        /// Starts the Kinect device and registers AllFramesReady(object source, AllFramesReadyEventArgs e) 
        /// event that fires when (all) frames are ready.
        /// 
        /// </summary>
        private void StartCameraDevice()
        {
            try
            {
                camDevice.Start();
                camDevice.AllFramesReady += CameraAllFramesReady;
            }
            catch (Exception exception)
            {
                Console.WriteLine("Error starting Kinect device: " + exception.ToString());
            }
        }

        /// <summary>
        /// 
        /// AllFramesReady event handler. Retrieves HandRight Joint position from Skeleton frame,  
        /// crops [CROP_WINDOW_WIDTH, CROP_WINDOW_HEIGHT] window around right hand joint and
        /// saves the cropped image as file.
        /// 
        /// </summary>
        /// <param name="source"></param>
        /// <param name="e"></param>
        private void CameraAllFramesReady(object source, AllFramesReadyEventArgs e)
        { 
            colorPixels = new byte[this.camDevice.ColorStream.FramePixelDataLength];
            float X = 0;
            float Y = 0;

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    Skeleton[] allSkeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(allSkeletons);

                    if (allSkeletons.Length >= 1)
                    {
                        foreach (Skeleton skeleton in allSkeletons)
                        {
                            if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                            {
                                activeSkeleton = skeleton;
                            }
                        }

                        if (activeSkeleton != null && activeSkeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            Joint rightHandJoint = activeSkeleton.Joints[JointType.HandRight];
                            X = Scale(640, 1.0f, rightHandJoint.Position.X); // check .5f
                            Y = Scale(480, 1.0f, -rightHandJoint.Position.Y); // check .5f
                            
                            Console.WriteLine("Joint Position: " + X + ", " + Y);
                        }
                    }
                }
            }

            using (ColorImageFrame colorImageFrame = e.OpenColorImageFrame())
            {
                if (colorImageFrame != null)
                {
                    colorImageFrame.CopyPixelDataTo(colorPixels);

                    this.colorBitmap = new WriteableBitmap(this.camDevice.ColorStream.FrameWidth, this.camDevice.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                    if (X != 0 && Y != 0)
                    {
                        // write pixels to bitmap
                        this.colorBitmap.WritePixels(
                          new Int32Rect((int)0, (int)0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight), 
                          this.colorPixels,
                          this.colorBitmap.PixelWidth * sizeof(int),
                          0);                    
                      
                        // crop the image
                        CroppedBitmap cropped = new CroppedBitmap(
                            colorBitmap, 
                            new Int32Rect(
                              (int)X - CROP_WINDOW_WIDTH / 2, (int)Y - Convert.ToInt32(CROP_WINDOW_HEIGHT / 1.5), CROP_WINDOW_WIDTH, CROP_WINDOW_HEIGHT));

                        // save to file
                        counter++;
                        SaveWritableBitmapToFile("test-image-" + counter + ".jpg", cropped.Clone());
                    }
                }
            }
        }
        
        /// <summary>
        /// 
        /// Saves the bitmap as file using file stream.
        /// 
        /// </summary>
        /// <param name="filename">Filename with extension</param>
        /// <param name="image">Image to be saved</param>
        private void SaveWritableBitmapToFile(string filename, BitmapSource image)
        {
            if (filename != string.Empty)
            {
                using (FileStream stream = new FileStream(filename, FileMode.Create))
                {
                    PngBitmapEncoder encoder = new PngBitmapEncoder();
                    encoder.Frames.Add(BitmapFrame.Create(image));
                    encoder.Save(stream);
                    stream.Close();
                }
            }
        }

        /// <summary>
        /// 
        /// Scales a float number(the position of a joint) to <paramref name="maxPixel"/> range from <paramref name="maxSkleton"/> range.
        /// 
        /// </summary>
        /// <param name="maxPixel">Scale range</param>
        /// <param name="maxSkeleton">Skeleton scale range</param>
        /// <param name="position">Value to be scaled</param>
        /// <returns></returns>
        private static float Scale(int maxPixel, float maxSkeleton, float position)
        {
            float value = ((((maxPixel / maxSkeleton) / 2) * position) + (maxPixel / 2));
            
            if (value > maxPixel)
                return maxPixel;
            if (value < 0)
                return 0;
           
            return value;
        }
    }
}
