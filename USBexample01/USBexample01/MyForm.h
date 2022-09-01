#include "usb.h"

#pragma once

extern int USB_packet_received;
extern std::string readbuf;

namespace USBexample01 
{
    //void WINAPI mainThread(LPVOID);
    //public: System::Void WINAPI mainThread(LPVOID);

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Threading; //включаем поддержку работы с потоками

	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{

	//private: System::Windows::Forms::Button^  button3;
	public:
		Thread ^ readThread;  //создаем экземпляр потока, в котором будем следить за приемом
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  button1;
	protected:
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::CheckBox^  checkBox1;
	private: System::Windows::Forms::CheckBox^  checkBox2;
	private: System::Windows::Forms::CheckBox^  checkBox3;
	private: System::Windows::Forms::CheckBox^  checkBox4;
	private: System::Windows::Forms::ListBox^  listBox1;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->checkBox1 = (gcnew System::Windows::Forms::CheckBox());
			this->checkBox2 = (gcnew System::Windows::Forms::CheckBox());
			this->checkBox3 = (gcnew System::Windows::Forms::CheckBox());
			this->checkBox4 = (gcnew System::Windows::Forms::CheckBox());
			this->listBox1 = (gcnew System::Windows::Forms::ListBox());
			this->SuspendLayout();
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(44, 33);
			this->button1->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(185, 53);
			this->button1->TabIndex = 0;
			this->button1->Text = L"Connect";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// button2
			// 
			this->button2->Enabled = false;
			this->button2->Location = System::Drawing::Point(40, 107);
			this->button2->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(189, 58);
			this->button2->TabIndex = 1;
			this->button2->Text = L"Disconnect";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &MyForm::button2_Click);
			// 
			// checkBox1
			// 
			this->checkBox1->AutoSize = true;
			this->checkBox1->Enabled = false;
			this->checkBox1->Location = System::Drawing::Point(271, 39);
			this->checkBox1->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->checkBox1->Name = L"checkBox1";
			this->checkBox1->Size = System::Drawing::Size(62, 20);
			this->checkBox1->TabIndex = 2;
			this->checkBox1->Text = L"Led 1";
			this->checkBox1->UseVisualStyleBackColor = true;
			this->checkBox1->CheckedChanged += gcnew System::EventHandler(this, &MyForm::checkBox1_CheckedChanged);
			// 
			// checkBox2
			// 
			this->checkBox2->AutoSize = true;
			this->checkBox2->Enabled = false;
			this->checkBox2->Location = System::Drawing::Point(271, 79);
			this->checkBox2->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->checkBox2->Name = L"checkBox2";
			this->checkBox2->Size = System::Drawing::Size(62, 20);
			this->checkBox2->TabIndex = 3;
			this->checkBox2->Text = L"Led 2";
			this->checkBox2->UseVisualStyleBackColor = true;
			this->checkBox2->CheckedChanged += gcnew System::EventHandler(this, &MyForm::checkBox2_CheckedChanged);
			// 
			// checkBox3
			// 
			this->checkBox3->AutoSize = true;
			this->checkBox3->Enabled = false;
			this->checkBox3->Location = System::Drawing::Point(271, 118);
			this->checkBox3->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->checkBox3->Name = L"checkBox3";
			this->checkBox3->Size = System::Drawing::Size(62, 20);
			this->checkBox3->TabIndex = 4;
			this->checkBox3->Text = L"Led 3";
			this->checkBox3->UseVisualStyleBackColor = true;
			this->checkBox3->CheckedChanged += gcnew System::EventHandler(this, &MyForm::checkBox3_CheckedChanged);
			// 
			// checkBox4
			// 
			this->checkBox4->AutoSize = true;
			this->checkBox4->Enabled = false;
			this->checkBox4->Location = System::Drawing::Point(271, 155);
			this->checkBox4->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->checkBox4->Name = L"checkBox4";
			this->checkBox4->Size = System::Drawing::Size(62, 20);
			this->checkBox4->TabIndex = 5;
			this->checkBox4->Text = L"Led 4";
			this->checkBox4->UseVisualStyleBackColor = true;
			this->checkBox4->CheckedChanged += gcnew System::EventHandler(this, &MyForm::checkBox4_CheckedChanged);
			// 
			// listBox1
			// 
			this->listBox1->FormattingEnabled = true;
			this->listBox1->ItemHeight = 16;
			this->listBox1->Location = System::Drawing::Point(40, 203);
			this->listBox1->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->listBox1->Name = L"listBox1";
			this->listBox1->Size = System::Drawing::Size(572, 164);
			this->listBox1->TabIndex = 6;
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(728, 430);
			this->Controls->Add(this->listBox1);
			this->Controls->Add(this->checkBox4);
			this->Controls->Add(this->checkBox3);
			this->Controls->Add(this->checkBox2);
			this->Controls->Add(this->checkBox1);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedSingle;
			this->Margin = System::Windows::Forms::Padding(4, 4, 4, 4);
			this->MaximizeBox = false;
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &MyForm::MyForm_FormClosed);
			this->Load += gcnew System::EventHandler(this, &MyForm::MyForm_Load);
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) 
	{
		if (enumd() == 0)
		  {
			this->button1->Enabled = false;
			this->button2->Enabled = true;
			this->checkBox1->Enabled = true;
			this->checkBox2->Enabled = true;
			this->checkBox3->Enabled = true;
			this->checkBox4->Enabled = true;
	      }
	}
private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) 
    {
	       disconhid();
		   this->button1->Enabled = true;
		   this->button2->Enabled = false;
		   this->checkBox1->Enabled = false;
		   this->checkBox2->Enabled = false;
		   this->checkBox3->Enabled = false;
		   this->checkBox4->Enabled = false;
    }
private: System::Void checkBox1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) 
    {
	  if (checkBox1->Checked)
	  {
		ledon(0x11);
	  }
	  else
	  {
		ledon(0x01);
	  }
    }
private: System::Void checkBox2_CheckedChanged(System::Object^  sender, System::EventArgs^  e) 
    {
	  if (checkBox2->Checked)
	    {
		  ledon(0x22);
	    }
	  else
	    {
		  ledon(0x02);
	    }
    }
private: System::Void checkBox3_CheckedChanged(System::Object^  sender, System::EventArgs^  e) 
    {
	  if (checkBox3->Checked)
	    {
		  ledon(0x33);
	    }
	  else
	    {
		  ledon(0x03);
	    }
    }
private: System::Void checkBox4_CheckedChanged(System::Object^  sender, System::EventArgs^  e) 
    {
	  if (checkBox4->Checked)
	   {
		ledon(0x44);
	   }
	  else
	   {
		ledon(0x04);
	   }
    }



//=================================================================================================
//Действия программы при загрузке окна формы
private: System::Void MyForm_Load(System::Object^  sender, System::EventArgs^  e) 
     {
	  //Последствия извращения мелкомягких над старым добрым C++
	  //Короче, создаем поток
	  ThreadStart^ myThreadDelegate = gcnew ThreadStart(this,&MyForm::Print);
	  readThread = gcnew Thread(myThreadDelegate);
	  readThread->Start(); //запускаем экземпляр потока, в котором слушаем линию на прием
     }
//=================================================================================================
//Действия программы при закрытии приложения
private: System::Void MyForm_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e) 
     { 
	  readThread->Abort(); //прерываем основной поток при выходе из программы
     }
//================================================================================================
//Показать информацию в Лист Боксе
public: void UpdateListBox(void)
{
	auto data_string = gcnew String(readbuf.c_str()); 
	listBox1->Items->Add(data_string); //печатаем принятую строку
}

//================================================================================================
//Основной поток. Выполняется вечно, и смотрит, пришло что либо (USB_packet_received будет =1)
public: void Print(void)
{
	while (1)
	{
		//Если заголовок окна создан и ещё не закрыт
		if (IsHandleCreated && !IsDisposed)
		{
			if (USB_packet_received == 1) //если данные по USB пришли
			{
				//запускаем функцию печати в Лист Бокс методом Инвоук
				this->Invoke(gcnew MethodInvoker(this, &MyForm::UpdateListBox));
				USB_packet_received = 0; // Сброс флага наличия пакета
			}
			
		}
	}
}
};
}
