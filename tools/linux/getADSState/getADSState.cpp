
#include "AdsLib.h"
#include "string.h"
#include <iostream>

long readStateExample(std::ostream& out, long port, const AmsAddr& server)
{
    uint16_t adsState;
    uint16_t devState;

    const long status = AdsSyncReadStateReqEx(port, &server, &adsState, &devState);
    if (status) {
        out << "ADS read failed with: " << std::dec << status << '\n';
        return -status;
    }
    out << "ADS state: " << std::dec << adsState << " devState: " << std::dec << devState << '\n';
    return adsState;	
}

long runExample(std::ostream& out, AmsNetId remoteNetId,  char remoteIpV4[])
{
    long nReturnVal=0;
    // add local route to your EtherCAT Master
    if (AdsAddRoute(remoteNetId, remoteIpV4)) {
       	out << "Adding ADS route failed, did you specified valid addresses?\n";
        return -1;
    }

    // open a new ADS port
    const long port = AdsPortOpenEx();
    if (!port) {
        out << "Open ADS port failed\n";
        return -2;
    }
    const AmsAddr remote { remoteNetId, AMSPORT_R0_PLC_TC3 };
    nReturnVal=readStateExample(out, port, remote);

    const long closeStatus = AdsPortCloseEx(port);
    if (closeStatus) {
        out << "Close ADS port failed with: " << std::dec << closeStatus << '\n';
    }
    return nReturnVal;
}

int main(int argc,char* argv[])
{
	//Program Returns ADS state
	std::string sRemoteNetID(40,'\n');
	std::ostream& outStream=std::cout;
        AmsNetId remoteNetId=sRemoteNetID;
        char* remoteIpV4;
        // build NetID from IP if only NetID not specified
	if (argc>3) //Argument for display text	
	{				
		if (atoi(argv[3])>0)
		{
			outStream.rdbuf(nullptr);
		}
	}
	if (argc>1)	
	{
	        remoteIpV4=argv[1]; //Second argument
		int ipLength=strlen(argv[1]);
		if(ipLength<16)
		{ 	
			int i=0;
			for(int i=0;i<ipLength;i++) 
			{
		   		sRemoteNetID[i]=argv[1][i];    
			}
			sRemoteNetID[i+1]='.';
			sRemoteNetID[i+2]='1';
			sRemoteNetID[i+3]='.';
			sRemoteNetID[i+4]='1';		
			remoteNetId=sRemoteNetID; 			
		}
		else
		{
       	        	outStream << "IP adress to long: " << remoteIpV4 << '\n';
			return -4;
		}

	}
	else
		if (argc>2)
		{
			//for(int i = 0; i < argc; i++)
		      	//	std::cout << "argv[" << i << "] = " << argv[i] << '\n';	
		        remoteIpV4=argv[1];
			sRemoteNetID=argv[2];
			remoteNetId=sRemoteNetID;
       		        outStream << "IP: " << remoteIpV4 << '\n';
      		        outStream << "AMS Net Id: " << remoteNetId << '\n';			
	        }
		else	
		{	
	       		outStream << "Argument missing: Please specify IP adress and AMS Net Id.\n";
			return -3;
		}
	//Call to actual function
       	return runExample(outStream,remoteNetId, remoteIpV4);
}
