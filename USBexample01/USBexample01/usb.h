#pragma once

#include <Windows.h>
#include <setupapi.h>
#include <stdio.h>
#include <devguid.h>
#include <regstr.h>
#include <tchar.h>
#include <string.h>
#include <devpropdef.h>
#include <devpkey.h>
#include <initguid.h>
#include <hidsdi.h>
#include <iostream>
#include <locale.h>
#include <io.h>
#include <fcntl.h>
#include <sys\stat.h>
#include <sys\types.h>
#include <stdlib.h>
#include <fstream>

int enumd(void);
int connecthid(std::wstring path);
int getinfo(std::wstring path);
int disconhid(void);
void senddata(void);
void ledon(unsigned char data);



