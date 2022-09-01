#include "usb.h"

#pragma comment(lib, "setupapi.lib")

//*****************************************************************************
//*****************************************************************************
//---------------------------------------------------------------------------
//******************************USB STREAMS *********************************
//----------------------------------------------------------------------------
//USB CONNECTION FUNC*********************************************************
DWORD WINAPI ReadThread(LPVOID);  // поток чтения
DWORD WINAPI WriteThread(LPVOID); // поток записи
HANDLE reader;               //поток чтения
HANDLE writer;              //объявляем поток записи
OVERLAPPED oRead;          //флаг асинхронного чтения
OVERLAPPED oWrite;         //флаг асинхронной записи (не используется)
HANDLE fileID = 0;  //Заголовок устройства
unsigned char reportwrite[65]; //буфер на запись длинной 65 байт
unsigned char reportread[65];  // буфер приема размером 65 байт (1служ + 64 данные)
							   //(1-й - служебный, =1)
							   //--Поток чтения READ THREAD--------------------------------------------------
							   //Читаем данные с устройства. Будет отправлено 65 байт.
							   //1-й байт служебный, с номером репорта (=1), остальные байты - данные

//======================================================================================================
int USB_packet_received = 0; //флаг принятого пакета. Если 1, то пакет принят
std::string readbuf;
int killRead=0;
//=======================================================================================================
DWORD WINAPI ReadThread(LPVOID)
{
	DWORD Bytes = 0;
	//создаем объект событие
	oRead.hEvent = CreateEvent(NULL,false,false,NULL);
	while (!killRead)
	{
		bool pr = false;
		pr = ReadFile(fileID, &reportread[0], 65, &Bytes, (LPOVERLAPPED)&oRead);
		//Ждем, пока не завершится чтение
		DWORD prw = WaitForSingleObject(oRead.hEvent, INFINITE);
		if (prw == WAIT_OBJECT_0) //если успешно
		{
			//заполняем буфер начиная с байта [1].  
			//Байт с индексом [0] содержит ID репорта, который для вывода в строку не нужен.
			readbuf = std::string((char*)&reportread[1]); //c байта [1]
			USB_packet_received = 1; //устанавливаем флаг приема
			prw = NULL;
		}
	}
	return 0;
}
//*****************************************************************************
// Поток записи WRITE THREAD-----------------------------------------------------
//Пишем данные к устройству. Размер буфера 65 байт (1-служебный (=2), остальные
//байты - данные
DWORD WINAPI WriteThread(LPVOID)
{
	DWORD bcount = 0;
	DWORD signal;
	bool pr = false;
	//создаем объект событие
	//oWrite.hEvent = CreateEvent(NULL,false,false,NULL);
	while (1)
	{
		pr = WriteFile(fileID, &reportwrite, 65, &bcount, &oWrite);
		//signal = WaitForSingleObject(oWrite.hEvent, INFINITE);//ждать завершения записи
		//if (signal == WAIT_OBJECT_0) //если запись успешно прошла
		//{
          //какой-то код
		//}
		SuspendThread(writer); //приостанавливаем поток после завершения операции
	}
}
//******************************************************************************
//******************************************************************************




//******************************************************************************
//******************** USB FUNCTIONS *******************************************

HDEVINFO hDev; //информация о списке устройств
SP_DEVINFO_DATA dInf; //массив данных об устройстве
SP_DEVICE_INTERFACE_DATA dIntDat; //массив данных об интерфейсе
PSP_DEVICE_INTERFACE_DETAIL_DATA    dIntDet = NULL;
ULONG                               pLength = 0;
ULONG                               rLength = 0;
DWORD i;
std::wstring devpath;
std::wstring stabp;
int flg;
int g = 0;

//*****************************************************************************
//Init functions
typedef void (WINAPI* pHidD_GetHidGuid)(OUT LPGUID);
typedef BOOLEAN(WINAPI* pHidD_GetManufacturerString)(IN HANDLE, OUT PVOID, IN ULONG);
typedef BOOLEAN(WINAPI* pHidD_GetProductString)(IN HANDLE, OUT PVOID, IN ULONG);
typedef BOOLEAN(WINAPI* pHidD_GetFeature)(IN HANDLE, OUT PVOID, IN ULONG);
typedef BOOLEAN(WINAPI* pHidD_SetFeature)(IN HANDLE, IN PVOID, IN ULONG);
typedef BOOLEAN(WINAPI* pHidD_SetOutputReport)(IN HANDLE, IN PVOID, IN ULONG);
typedef BOOLEAN(WINAPI* pHidD_GetAttributes)(HANDLE, PHIDD_ATTRIBUTES);
typedef BOOLEAN(WINAPI* pHidD_GetInputReport)(IN HANDLE, IN PVOID, IN ULONG);

bool d = true;

HINSTANCE hDLL = LoadLibrary(L"HID.DLL");
GUID hguid;

pHidD_GetProductString   GetProductString = NULL;
pHidD_GetHidGuid         GetHidGuid = NULL;
pHidD_GetAttributes      GetAttributes = NULL;
pHidD_SetFeature         SetFeature = NULL;
pHidD_GetFeature         GetFeature = NULL;
pHidD_SetOutputReport    SetOutReport = NULL;
pHidD_GetInputReport     GetInputReport = NULL;
//****************************************************************************

void loadlib()
{
	if (hDLL != 0)
	{
		GetHidGuid = (pHidD_GetHidGuid)GetProcAddress(hDLL, "HidD_GetHidGuid");
		GetProductString = (pHidD_GetProductString)GetProcAddress(hDLL, "HidD_GetProductString");
		GetFeature = (pHidD_GetFeature)GetProcAddress(hDLL, "HidD_GetFeature");
		SetFeature = (pHidD_SetFeature)GetProcAddress(hDLL, "HidD_SetFeature");
		GetAttributes = (pHidD_GetAttributes)GetProcAddress(hDLL, "HidD_GetAttributes");
		SetOutReport = (pHidD_SetOutputReport)GetProcAddress(hDLL, "HidD_SetOutputReport");
		if (GetHidGuid)
		{
			GetHidGuid(&hguid);
		}
	}
}


//************ENUMERATE USB DEVICES *****************************************
int enumd()
{
	int status = 100;
	flg = 0;
	loadlib();       //Грузим библиотеку
	i = 0;             //инкрементарный указатель обнуляем
	hDev = SetupDiGetClassDevs(&hguid, NULL, 0, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	if (hDev == INVALID_HANDLE_VALUE)
	{
		status = 1;
	}
	else
	{
		dInf.cbSize = sizeof(SP_DEVINFO_DATA);
		dIntDat.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
		//Прозваниваем каждое из устройств в цикле, пока они не закончатся
		//SetupDiEnumDeviceInterfaces(hDev,0,&hguid,i,&dIntDat) вернет FALSE
		while (SetupDiEnumDeviceInterfaces(hDev, 0, &hguid, i, &dIntDat))
		{
			if (dIntDet)
			{
				free(dIntDet);
				dIntDet = NULL;
			}
			SetupDiGetDeviceInterfaceDetail(hDev, &dIntDat, 0, 0, &rLength, 0);
			pLength = rLength;
			dIntDet = (SP_DEVICE_INTERFACE_DETAIL_DATA*)malloc(pLength);
			if (dIntDet)
				dIntDet->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
			if (!SetupDiGetDeviceInterfaceDetail(hDev, &dIntDat, dIntDet, rLength, &rLength, 0))
			{
				free(dIntDet);
			}
			devpath = dIntDet->DevicePath;
			//LPCWSTR p = dsval.c_str();
			//OutputDebugString(p);
			//OutputDebugString(L"\n");
			if (flg == 0)
			{
				if (connecthid(devpath) == 0) //пробуем соединиться.
				{
					status = 0; //если успешно
					break;
				}
			}
			i = i + 1;
		}
		//если ничего не нашли
		if (flg == 0)
		{
			status = 1;
		}
		SetupDiDestroyDeviceInfoList(hDev);
	}
	return status;
}

//********** CONNECT TO DEVICE ********************************************
int connecthid(std::wstring path)
{
	int status = 100;
	if (getinfo(path) == 0)
	{
		LPCWSTR pathusb = path.c_str();
		fileID = CreateFile(pathusb,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL, // no SECURITY_ATTRIBUTES structure
			OPEN_EXISTING, // No special create flags
			FILE_FLAG_OVERLAPPED, // No special attributes
			0);

		if (fileID == INVALID_HANDLE_VALUE)
		{
			status = 1;
		}
		else
		{
			status = 0;
			//Открываем поток на чтение. Он работает постоянно до закрытия
			//сеанса связи
			reader = CreateThread(NULL, 0, ReadThread, NULL, CREATE_ALWAYS, NULL);
			//создаем поток записи в приостановленном состоянии.
			writer = CreateThread(NULL, 0, WriteThread, NULL, CREATE_SUSPENDED, NULL);
			//Отчитываемся об успешном подключении и т.д.
			flg = 1;
			//disconhid(); // на всяки случай отключаем "не то" устройство
			//если оно вдруг открылось с CreateFile
		}
	}
	return status;
}

//*********************************************************************************
//****GET INFO ABOUT DEVICE********************************************************
int getinfo(std::wstring path)
{
	int status = 100;
	const std::string fullpath(path.begin(), path.end());
	const char *fullp = fullpath.c_str();
	char vids[5] = { 0,0,0,0,0 };
	char pids[5] = { 0,0,0,0,0 };
	strncpy(vids, &fullp[12], 4);
	strncpy(pids, &fullp[21], 4);
	//_RPT1(0, "%s\n", vids);
	//_RPT1(0, "%s\n", pids);
	if (strncmp(vids, "0483", 4) == 0)
	{
		if (strncmp(pids, "5762", 4) == 0)
		{
			//OutputDebugString(L"OKEY");
			status = 0;
		}
	}
	return status;
}

//****************************************************************************
//---------------------------------------------------------------------------
//DISCONNECT DEVICE
int disconhid()
{
	int status = 100;
	//если выполняется поток чтения - отключаем его
	if (reader)
	{
		killRead = 1;
		TerminateThread(reader, 0);
		//CloseHandle(oRead.hEvent);	//нужно закрыть объект-событие
		//CloseHandle(reader);
	}
	//Если работает поток на запись - тоже отключаем
	if (writer)
	{
		TerminateThread(writer, 0);
		CloseHandle(oWrite.hEvent);	//нужно закрыть объект-событие
		CloseHandle(writer);
	}
	//отключаем устройство
	CloseHandle(fileID);
	status = 0;
	flg = 0;
	return status;
}

//***************************************************************************
//SEND DATA TO DEVICE ----------------------------------------------------------
void senddata()//(unsigned char *sBuf, int datalen)
{
	//отправляем данные. Включается поток записи.
	//После завершения записи, поток опять приостанавливается
	//Данные представляют собой массив из 65 байт,
	//где 1-й байт - ID репорта на запись (=2)
	//Отправляем данные. Просто открываем поток WriteThread
	ResumeThread(writer);
}


void ledon(unsigned char data)
{
	reportwrite[0] = 0x02;
	reportwrite[1] = data;
	senddata();
}


//********************************************************************