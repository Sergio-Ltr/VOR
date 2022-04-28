#define _USE_MATH_DEFINES
 
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <iostream>
#include <time.h>
#include <string>
#include <unistd.h>
#include <array>
#include <cmath>

using namespace yarp::os;
using namespace yarp::sig; 
using namespace std;

unsigned sleep (unsigned seconds);

class EyeMover : public PeriodicThread {
    public:
        EyeMover(double p, double a, double f, bool g = false) : 
        PeriodicThread(p){
            //Initialize variables.
            periodCounter = 0;
            period = p;
            amplitude = a;
            frequence = f;
            staticGain = g;

            //Connect to necessary ports:

            // Declare iCub robot head port name.
            string headPortName = "/icubSim/head/rpc:i";
            string headClientName = "/VOR/head-controller";

            //Open connection to head port.
            bool headOk = head.open(headClientName);
            
            //Check of the connection has been established successfully.
            if(!headOk) {
                std::cerr << "Failed to conncect to head port, ensure iCub simulator is running." << std::endl;
            }

            //Connect RPC Client to iCub head RPC Server.
            yarp::os::Network::connect(headClientName, headPortName);

            // Declare iCub robot encoders port name.
            string encodersPortName = "/icubSim/head/state:o";
            string encodersClientName = "/VOR/encoders";

            //Conncect to head port in order to read values from encoders port.
            bool encodersOk = encodersPort.open(encodersClientName);

            //Check of the connection has been established successfully.
            if(!encodersOk) {
                std::cerr << "Failed to conncect to encoders port, ensure iCub simulator is running." << std::endl;
            }

            //Connect the buffered ports.
            yarp::os::Network::connect(encodersPortName, encodersClientName);

            string targetPortName = "/VOR/vision";
            string sphereCenterPortName = "/VOR/sphere-center"; 

            bool sphereCenterPortOk = sphereCenterPort.open(sphereCenterPortName);

            //Check of the connection has been established successfully.
            if(!encodersOk) {
                std::cerr << "Failed to conncect to sphere port, ensure iCub simulator is running." << std::endl;
            }

            //Connect the buffered ports.
            yarp::os::Network::connect(targetPortName, sphereCenterPortName);
        }
    
        bool threadInit() override
        {
            //Nothing particular here, just logging.
            std::cout << "Starting moving eyes" << "\n";
            return true;
        }
    
        void afterStart(bool s) override
        {
            //Nothing particular here, just logging.
            if (s) {
                std::cout << "Eye mover thread started successfully" << "\n";
            } else {
                std::cerr  << "Eye mover thread did not start" << "\n";
            }
        }
    
        void run() override
        {
            //Compute movement according to parameters.
            double sinInput = computeSin(periodCounter * period, amplitude, frequence);
            
            //Actually move head joints.
            moveHead(0, sinInput);
            moveHead(2, sinInput);

            //Count number of periods.
            periodCounter ++;

            //Get the current values of the eye joint encoders, using the head state port. 
            Vector *encoders = encodersPort.read();

            double vEyeEnc = (*encoders)[3];
            double hEyeEnc = (*encoders)[4];

            // Get estimated center  coordinates from the buffered port we created in the vision thread.
            Vector *sphere = sphereCenterPort.read();

            // Use the last element of the vector to check if sphere ceneter was computed. 
            if((*sphere)[2] != 0){
                double xSphereCenter = (*sphere)[0];
                double ySphereCenter = (*sphere)[1];

                // Implementing the close loop controller here. 
                int xDiff = 160 - xSphereCenter;
                int yDiff = 120 - ySphereCenter;

                double gainFactor = 0.1; //In case we do not scale by freq, we need a 10^-1 factor. 

                //Scale gain (and so error correction) according to movement frequence.
                //If user impose a static gain, then this skip will be0 skipped.
                if(!staticGain){
                    gainFactor = frequence;
                }

                //We change gain according to osccilation frequence. 
                double kpX = -3.6 * gainFactor; //We need negative gain for horiziontal stability.
                double kpY = 1.8 * gainFactor; 
                
                // Sum u(t) and current encoders position.
                moveHead(4, xDiff * kpX + hEyeEnc);
                moveHead(3, yDiff * kpY + vEyeEnc);
            }
        }
    
        void threadRelease() override
        {
            //Reset eye position to 0.
            moveHead(3,0);
            moveHead(4,0);

            //Close ports used by this thread.
            head.close();
            encodersPort.close();
            sphereCenterPort.close();
            std::cout << "Goodbye from eye mover thread" << "\n";
        };

    private: 
         //Attributes:
        double period; //Movement frequency.
        int periodCounter; //Number of the current period.

        double amplitude; //Amplitude of the movement.
        double frequence; //Frequence of the movement.

        bool staticGain;  //The way user choose to hold the Kps.

        RpcClient head; //Attribute to trigger the eye movement.
        BufferedPort<Vector> sphereCenterPort; //Attribute to get robot vision.
        BufferedPort<Vector> encodersPort; //Attribute to get encoders positions. 

        //Methods:

        // Execute the move command for one of the joints in the iCub Robot head. 
        void moveHead(int joint, double value) {

            //Create a bottle containing the "SET POS" command.
            Bottle req;
            req.addString("set");
            req.addString("pos");
            req.addInt32(joint);
            req.addFloat64(value);
            
            //Send the command
            head.write(req);
        }

        // Compute the value of a Sinusoidal function.
        double computeSin(double t,double ampl,double freq, double phase = 0){
            // Assume phase to be 0 
            return ampl*sin(2 * M_PI * t * freq + phase);
        }
};

class RobotVision : public PeriodicThread {
    public: 
        RobotVision(double p):
        PeriodicThread(p){
            //Conncect to left camera in order tp receive images from the left camera. 
            bool cameraOk = imagePort.open("/VOR/image-receiver");
            
            //Check if the connection has been established successfully.
            if(!cameraOk) {
                std::cerr << "Failed to conncect to left camera port, ensure iCub simulator is running." << std::endl;
            } 

            //Connect the buffered ports.
            yarp::os::Network::connect("/icubSim/cam/left", "/VOR/image-receiver");

            //Open the server port we will use to provide estimated sphere center coordinates.
            targetPort.open("/VOR/vision");
        } 
    
        bool threadInit() override
        {
            //Nothing particular here, just logging.
            std::cout << "Starting Robot vision" << "\n";
            return true;
        }

        void afterStart(bool s) override
        {
            //Nothing particular here, just logging.
            if (s) {
                std::cout << "Robot vision thread started successfully" << "\n";
            } else {
                std::cerr  << "Robot vision thread did not start" << "\n";
            }
        }
    
        void run() override
        {
            //Use third element as a "0/1 control value".
            array<int, 3> center = getSphereCenter();
           
            //Write the position into a sendable "Vector" object.
            Vector& target = targetPort.prepare();
            target.resize(3);
            target[0] = center[0];
            target[1] = center[1];
            target[2] = center[2];

            //Send the center position to the dedicated port. 
            targetPort.write();
        }
    
        void threadRelease() override
        {
            //Close ports used by this thread.
            imagePort.close();
            targetPort.close();
            std::cout << "Goodbye from robot vision thread" << "\n";
        }

    private: 
        //Attributes:
        BufferedPort<ImageOf<PixelRgb>> imagePort;
        BufferedPort<Vector> targetPort;
        
        //Iniyial sphere position is fixed.
        VectorOf<int> initial_sphere_center[2] = { 160, 120 };

        //Methods:

        // Get sphere center, first two elements are the coordinate, last one is a check. 
        array<int, 3> getSphereCenter(){ 
            //We shpould ask image to eye port here
            ImageOf<PixelRgb> *image = imagePort.read();  // read an image

            // If something is visible
            if (image != nullptr) {

                int xCount = 0;
                int yCount = 0;
                int pixelCount = 0;

                //Iterate over pixels within the image.
                for (int x=0; x<image->width(); x++){
                    for (int y=0; y<image->height(); y++) {
                        //Check if any pixel is red (belongs to sphere). 
                        PixelRgb& pixel = image -> pixel(x,y);
                        //Sphere color is something like (134,0,0). So we check... 
                        if(pixel.r > 100 & pixel.b < 10 & pixel.g < 10){
                            xCount += x;
                            yCount += y;
                            pixelCount ++;
                            // std:cout << "Red Pixel here";
                        }
                    }
                }

                // Not necessary in a percetly working system. Useful when trying different Kps,
                // in order to avoid division by zero when the sphere is out of our view. 
                if(pixelCount > 20 ){ 
                    //std::cout << "Estimed Center" << xCount/pixelCount << " - " << yCount/pixelCount << "\n";
                    return { int(xCount/pixelCount),int(yCount/pixelCount), 1 };
                }
            } 

            // If something goes wrong return this "error triple".
            // Error will be handled accoridng to clients' logic. 
            return {0,0,0};
        }
};

int main(int argc, char *argv[]) {
    //Check if user imposed static gain constants.
    bool staticGain = false;
    
    if(argc > 1 && argv[1] == string("-sg")){
    	staticGain = true;
    	std::cout << "Static gain imposed for the controler." << "\n";
    } else {
    	std::cout << "Scaling gain according to frequence variation." << "\n";
    }
  
    //Frequency of movement
    double MOVEMENT_FREQ = 0.05; 

    // Declare constants for the scheduled movements.
    array<double, 3> FREQUENCIES_HZ = { 0.1, 0.2, 0.4 };
    int AMPLITUDES_DEG = 16;
    int LATENCY_SEC = 15;

    // Declare Yarp
    Network yarp; 

    // Delcare iCub world port name. 
    string worldPortName = "/icubSim/world";
    string worldClientName = "/VOR/world-controller";

    //Open connection to head port.
    RpcClient worldRpc;
    bool worldOk = worldRpc.open(worldClientName);

    //Check of the connection has been established successfully.
    if(!worldOk) {
        std::cerr << "Failed to conncect to world port, ensure iCub simulator is running." << std::endl;
        return 1;
    }

    //Connect RPC Client to iCub world RPC Server.
    yarp::os::Network::connect(worldClientName, worldPortName);

    //Create the red sphere
    Bottle req;

    req.addString("world");
    req.addString("mk");
    req.addString("ssph");
    req.addFloat64(0.04);
    req.addFloat64(0.0);
    req.addFloat64(0.9);
    req.addFloat64(0.8);
    req.addInt32(1);
    req.addInt32(0);
    req.addInt32(0);

    worldRpc.write(req);

    // We should not need this port anymore. 
    worldRpc.close();
    
    RobotVision vision(MOVEMENT_FREQ);
    vision.start();

    // Loop through scheduled movements.
    for(size_t i = 0; i < FREQUENCIES_HZ.size(); i++){
        double freq = FREQUENCIES_HZ[i];

        // Inform user about what is happeing. 
        std::cout << "Movement - " << i + 1 << "\n";
        std::cout << "Amplitude: " << AMPLITUDES_DEG << "\n";
        std::cout << "Frequence: " << freq << "\n";

        //Create the thred to move eyes.
        EyeMover eyeMover(MOVEMENT_FREQ, AMPLITUDES_DEG, freq, staticGain);
        
        //Begin moving eyes.
        eyeMover.start();

        // Iteration should last exactly the scheduled amount of time. 
        sleep(LATENCY_SEC);
 
        //Thread should stop here
        eyeMover.stop();
        eyeMover.threadRelease();

        std::cout << "End of iteration.\n";

        //Stop for some secs within each iteration. 
        sleep(2);
    }

    //Stop threads.
    vision.stop();
    vision.threadRelease();

    return 0; 
}

