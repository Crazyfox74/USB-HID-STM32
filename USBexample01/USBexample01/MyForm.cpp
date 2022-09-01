#include "MyForm.h"


//==============================================================================
using namespace System;
using namespace System::Windows::Forms;
using namespace USBexample01;



[STAThreadAttribute]
void Main(array<String^>^ args)
{
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	USBexample01::MyForm form;
	Application::Run(%form);
}

//==============================================================================

