#define _WINSOCK_DEPRECATED_NO_WARNINGS

#pragma comment(lib,"ws2_32.lib")
#include <WinSock2.h>
#include <string>
#include <iostream>
#include <time.h>

#include <GL/GL.h>
#include <GL/glut.h>

#include <HL/hl.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>

#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

SOCKET Connection;


struct PointMass
{
    hduVector3Dd m_position;
    hduVector3Dd m_velocity;
    HDdouble m_mass; 
    HDdouble m_kStiffness;
    HDdouble m_kDamping;
};


void ClientThread()
{

	//std::string buffer;
	//int BufferLength;
	char send_buffer [1024];
	char rec_buffer[1024];
	int j;
	int counter=0;
	float fp_x=3.13;
	float fp_y=4.13;
	float fp_z=5.12;
	
	srand (static_cast <unsigned> (time(0)));
	clock_t begin = clock();

	while (counter<10000) 
	{

		
		memset(send_buffer, 0, sizeof(send_buffer));
		fp_x =  static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		fp_y =  static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		fp_z =  static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		j = sprintf_s(send_buffer,1024,"%f\n",fp_x);
	    j += sprintf_s(send_buffer+j,1024-j,"%f\n",fp_y);
	    j += sprintf_s(send_buffer+j,1024-j,"%f\n",fp_z);
		
		send(Connection, send_buffer, sizeof(send_buffer), NULL);

		if ((strncmp(send_buffer, "exit", 4)) == 0) 
		{
			std::cout << "Client Exit...\n" << std::endl;
			break;
		}
		//memset(buffer, 0, sizeof(buffer));
		recv(Connection, rec_buffer, sizeof(rec_buffer), NULL);
		
		counter++;
	}


	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout<<"Elapsed seconds after 10000 rounds:"<<elapsed_secs<<" seconds"<<std::endl;
	memset(send_buffer, 0, sizeof(send_buffer));
	sprintf_s(send_buffer,1024,"%s","exit");
	send(Connection, send_buffer, sizeof(send_buffer), NULL);
	std::cout << "Client Exit...\n" << std::endl;
	return;

}


void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata)
{
    PointMass *pPointMass = static_cast<PointMass *>(userdata);

    // Get the time delta since the last update.
    HDdouble instRate;
    hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
    HDdouble deltaT = 1.0 / instRate;
    
    // Get the current proxy position from the state cache.
    // Note that the effect state cache is maintained in workspace coordinates,
    // so we don't need to do any transformations in using the proxy
    // position for computing forces.
    hduVector3Dd proxyPos;
    hlCacheGetDoublev(cache, HL_PROXY_POSITION, proxyPos);
    
    // Compute the inertia force based on pulling the point mass around
    // by a spring.
    hduVector3Dd springForce = pPointMass->m_kStiffness * 
        (proxyPos - pPointMass->m_position);
    hduVector3Dd damperForce = -pPointMass->m_kDamping * pPointMass->m_velocity;
    hduVector3Dd inertiaForce = springForce + damperForce;
        
    // Perform Euler integration of the point mass state.
    hduVector3Dd acceleration = inertiaForce / pPointMass->m_mass;
    pPointMass->m_velocity += acceleration * deltaT;    
    pPointMass->m_position += pPointMass->m_velocity * deltaT;
                                  
    // Send the opposing force to the device.
    force[0] += -inertiaForce[0];
    force[1] += -inertiaForce[1];
    force[2] += -inertiaForce[2];
}


/******************************************************************************
 Servo loop thread callback called when the effect is started.
******************************************************************************/
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata)
{
    PointMass *pPointMass = (PointMass *) userdata;
    
    fprintf(stdout, "Custom effect started\n");

    // Initialize the position of the mass to be at the proxy position.
    hlCacheGetDoublev(cache, HL_PROXY_POSITION, pPointMass->m_position);

    pPointMass->m_velocity.set(0, 0, 0);
}


/******************************************************************************
 Servo loop thread callback called when the effect is stopped.
******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata)
{
    fprintf(stdout, "Custom effect stopped\n");
}


/******************************************************************************
 Initializes the control parameters used for simulating the point mass.
******************************************************************************/
void initPointMass(PointMass *pPointMass)
{
    pPointMass->m_mass = 0.001; // Kg        

    // Query HDAPI for the max spring stiffness and then tune it down to allow
    // for stable force rendering throughout the workspace.
    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &pPointMass->m_kStiffness);
    pPointMass->m_kStiffness *= 0.5;

    // Compute damping constant so that the point mass motion is
    // critically damped.
    pPointMass->m_kDamping = 2 * sqrt(pPointMass->m_mass *
                                      pPointMass->m_kStiffness);
}

int main() 
{
	// Winsock Startup
	WSAData wsaData;
	WORD DllVersion = MAKEWORD(2, 1);

	if (WSAStartup(DllVersion, &wsaData) != 0)
	{
		MessageBoxA(NULL, "Winsock startup failed", "Error", MB_OK | MB_ICONERROR);
		exit(1);
	}

	SOCKADDR_IN Address; // Address to be binded to our Connection socket
	int AddressSize = sizeof(Address); // Need AddressSize for the connect function below
	Address.sin_addr.s_addr = inet_addr("127.0.0.1"); // local IP Address
	Address.sin_port = htons(5555); // Port
	Address.sin_family = AF_INET; // Defines type of socket (IPv4 Socket)

	// Create socket and bind address
	Connection = socket(AF_INET, SOCK_STREAM, NULL);

	if (connect(Connection, (SOCKADDR*)&Address, AddressSize))
	{
		MessageBoxA(NULL, "Failed to Connect", "Error", MB_OK | MB_ICONERROR);
		return 0;
	}

	std::cout << "Connected to server" << std::endl;


	//Haptics initialization

	HHD hHD;
    HHLRC hHLRC;
    HDErrorInfo error;

    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        return -1;
    }
    hdMakeCurrentDevice(hHD);

    hHLRC = hlCreateContext(hHD);
    hlMakeCurrent(hHLRC);
    
    hlDisable(HL_USE_GL_MODELVIEW);

	 HLuint effect = hlGenEffects(1);        

    // Initialize the point mass.
    PointMass pointMass;
    initPointMass(&pointMass);

    hlBeginFrame();

    hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc) computeForceCB, &pointMass);
    hlCallback(HL_EFFECT_START, (HLcallbackProc) startEffectCB, &pointMass);
    hlCallback(HL_EFFECT_STOP, (HLcallbackProc) stopEffectCB, &pointMass);

    hlStartEffect(HL_EFFECT_CALLBACK, effect);

    hlEndFrame();

    fprintf(stdout, "Press any key to stop the effect\n");
    while (!_kbhit())
    {
        // Check for any errors.
        HLerror error;
        while (HL_ERROR(error = hlGetError()))
        {
            fprintf(stderr, "HL Error: %s\n", error.errorCode);
            
            if (error.errorCode == HL_DEVICE_ERROR)
            {
                hduPrintError(stderr, &error.errorInfo,
                    "Error during haptic rendering\n");
            }
        }                  
    }

    hlBeginFrame();
    hlStopEffect(effect);
    hlEndFrame();

    fprintf(stdout, "Shutting down...\n");
    getchar();

    hlDeleteEffects(effect, 1);

    hlDeleteContext(hHLRC);
    hdDisableDevice(hHD);


   
 

	//ClientThread();

	//system("pause");

	return 0;
}