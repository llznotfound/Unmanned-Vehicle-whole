#include "SerialPort.h"
#include <process.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;

/** �߳��˳���־ */
bool CSerialPort::s_bExit = false;
/** ������������ʱ,sleep���´β�ѯ����ʱ��,��λ:�� */
const UINT SLEEP_TIME_INTERVAL = 5;

CSerialPort::CSerialPort(void)
	: m_hListenThread(INVALID_HANDLE_VALUE)
{
	m_hComm = INVALID_HANDLE_VALUE;
	m_hListenThread = INVALID_HANDLE_VALUE;

	InitializeCriticalSection(&m_csCommunicationSync);

}

CSerialPort::~CSerialPort(void)
{
	CloseListenTread();
	ClosePort();
	DeleteCriticalSection(&m_csCommunicationSync);
}

bool CSerialPort::InitPort(UINT portNo /*= 1*/, UINT baud /*= CBR_9600*/, char parity /*= 'N'*/,
	UINT databits /*= 8*/, UINT stopsbits /*= 1*/, DWORD dwCommEvents /*= EV_RXCHAR*/)
{
	/** ��ʱ����,���ƶ�����ת��Ϊ�ַ�����ʽ,�Թ���DCB�ṹ */
	char szDCBparam[50];
	sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);

	/** ��ָ������,�ú����ڲ��Ѿ����ٽ�������,�����벻Ҫ�ӱ��� */
	if (!openPort(portNo))
	{
		return false;
	}

	/** �����ٽ�� */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �Ƿ��д����� */
	BOOL bIsSuccess = TRUE;

	/** �ڴ˿���������������Ļ�������С,���������,��ϵͳ������Ĭ��ֵ.
	*  �Լ����û�������Сʱ,Ҫע�������Դ�һЩ,���⻺�������
	*/
	/*if (bIsSuccess )
	{
	bIsSuccess = SetupComm(m_hComm,10,10);
	}*/

	/** ���ô��ڵĳ�ʱʱ��,����Ϊ0,��ʾ��ʹ�ó�ʱ���� */
	COMMTIMEOUTS  CommTimeouts;
	CommTimeouts.ReadIntervalTimeout = 0;
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant = 0;
	if (bIsSuccess)
	{
		bIsSuccess = SetCommTimeouts(m_hComm, &CommTimeouts);
	}

	DCB  dcb;
	if (bIsSuccess)
	{
		// ��ANSI�ַ���ת��ΪUNICODE�ַ���
		DWORD dwNum = MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t *pwText = new wchar_t[dwNum];
		if (!MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, pwText, dwNum))
		{
			bIsSuccess = TRUE;
		}

		std::wstring s = pwText;
		const int size = ::WideCharToMultiByte(CP_UTF8, 0, s.c_str(), -1, NULL, 0, 0, NULL);
		std::vector<char> buf(size);
		::WideCharToMultiByte(CP_UTF8, 0, s.c_str(), -1, &buf[0], size, 0, NULL);
		std::string utf8String =  std::string(&buf[0]);
		LPCSTR lpStr = utf8String.c_str();

		/** ��ȡ��ǰ�������ò���,���ҹ��촮��DCB���� */
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(lpStr, &dcb);
		/** ����RTS flow���� */
		dcb.fRtsControl = RTS_CONTROL_ENABLE;

		/** �ͷ��ڴ�ռ� */
		delete[] pwText;
	}

	if (bIsSuccess)
	{
		/** ʹ��DCB�������ô���״̬ */
		bIsSuccess = SetCommState(m_hComm, &dcb);
	}

	/**  ��մ��ڻ����� */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** �뿪�ٽ�� */
	LeaveCriticalSection(&m_csCommunicationSync);

	return bIsSuccess == TRUE;
}

bool CSerialPort::InitPort(UINT portNo, const LPDCB& plDCB)
{
	/** ��ָ������,�ú����ڲ��Ѿ����ٽ�������,�����벻Ҫ�ӱ��� */
	if (!openPort(portNo))
	{
		return false;
	}

	/** �����ٽ�� */
	EnterCriticalSection(&m_csCommunicationSync);

	/** ���ô��ڲ��� */
	if (!SetCommState(m_hComm, plDCB))
	{
		return false;
	}

	/**  ��մ��ڻ����� */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	/** �뿪�ٽ�� */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

void CSerialPort::ClosePort()
{
	/** ����д��ڱ��򿪣��ر��� */
	if (m_hComm != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hComm);
		m_hComm = INVALID_HANDLE_VALUE;
	}
}

bool CSerialPort::openPort(UINT portNo)
{
	/** �����ٽ�� */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �Ѵ��ڵı��ת��Ϊ�豸�� */
	char szPort[50];
	/** �����������ںŴ���10���������"COM12"��"COM20"�����ģ��򿪴���(createfile)ʱ�����׷��һ��ǰ׺��"\\\\.\\"�� */
	sprintf_s(szPort, "\\\\.\\COM%d", portNo);

	/** ��ָ���Ĵ��� */
	m_hComm = CreateFileA(szPort,  /** �豸��,COM1,COM2�� */
		GENERIC_READ | GENERIC_WRITE, /** ����ģʽ,��ͬʱ��д */
		0,                            /** ����ģʽ,0��ʾ������ */
		NULL,                         /** ��ȫ������,һ��ʹ��NULL */
		OPEN_EXISTING,                /** �ò�����ʾ�豸�������,���򴴽�ʧ�� */
		0,
		0);

	/** �����ʧ�ܣ��ͷ���Դ������ */
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}

	/** �˳��ٽ��� */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

bool CSerialPort::OpenListenThread()
{
	/** ����߳��Ƿ��Ѿ������� */
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** �߳��Ѿ����� */
		return false;
	}

	s_bExit = false;
	/** �߳�ID */
	UINT threadId;
	/** ��������ݼ����߳� */
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
	if (!m_hListenThread)
	{
		return false;
	}
	/** �����̵߳����ȼ�,������ͨ�߳� */
	if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))
	{
		return false;
	}

	return true;
}

bool CSerialPort::CloseListenTread()
{
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** ֪ͨ�߳��˳� */
		s_bExit = true;

		/** �ȴ��߳��˳� */
		Sleep(10);

		/** ���߳̾����Ч */
		CloseHandle(m_hListenThread);
		m_hListenThread = INVALID_HANDLE_VALUE;
	}
	return true;
}

UINT CSerialPort::GetBytesInCOM()
{
	DWORD dwError = 0;  /** ������ */
	COMSTAT  comstat;   /** COMSTAT�ṹ��,��¼ͨ���豸��״̬��Ϣ */
	memset(&comstat, 0, sizeof(COMSTAT));

	UINT BytesInQue = 0;
	/** �ڵ���ReadFile��WriteFile֮ǰ,ͨ�������������ǰ����Ĵ����־ */
	if (ClearCommError(m_hComm, &dwError, &comstat))
	{
		BytesInQue = comstat.cbInQue; /** ��ȡ�����뻺�����е��ֽ��� */
	}

	return BytesInQue;
}

UINT WINAPI CSerialPort::ListenThread(void* pParam)
{
	/** �õ������ָ�� */
	CSerialPort *pSerialPort = reinterpret_cast<CSerialPort*>(pParam);

	// �߳�ѭ��,��ѯ��ʽ��ȡ��������
	while (!pSerialPort->s_bExit)
	{
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** ����������뻺������������,����Ϣһ���ٲ�ѯ */
		if (BytesInQue == 0)
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}

		/** ��ȡ���뻺�����е����ݲ������ʾ */
		char cRecved = 0x00;
		do
		{
			cRecved = 0x00;
			if (pSerialPort->ReadChar(cRecved) == true)
			{
				std::cout << cRecved;
				continue;
			}
		} while (--BytesInQue);
	}

	return 0;
}

bool CSerialPort::ReadChar(char &cRecved)
{
	BOOL  bResult = TRUE;
	DWORD BytesRead = 0;
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** �ٽ������� */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �ӻ�������ȡһ���ֽڵ����� */
	bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);
	if ((!bResult))
	{
		/** ��ȡ������,���Ը��ݸô�����������ԭ�� */
		DWORD dwError = GetLastError();

		/** ��մ��ڻ����� */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** �뿪�ٽ��� */
	LeaveCriticalSection(&m_csCommunicationSync);

	return (BytesRead == 1);

}

bool CSerialPort::WriteData(uint8_t pData, unsigned int length)
{
	BOOL   bResult = TRUE;
	DWORD  BytesToSend = 0;
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	/** �ٽ������� */
	EnterCriticalSection(&m_csCommunicationSync);

	/** �򻺳���д��ָ���������� */
	bResult = WriteFile(m_hComm, &pData, length, &BytesToSend, NULL);
	if (!bResult)
	{
		DWORD dwError = GetLastError();
		/** ��մ��ڻ����� */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);

		return false;
	}

	/** �뿪�ٽ��� */
	LeaveCriticalSection(&m_csCommunicationSync);

	return true;
}

bool CSerialPort::WriteFrame(Frame f)
{
	if (!WriteData(f.start, 1))	return FALSE;
	if (!WriteData(f.ctl_flag, 1)) return FALSE;
	if (!WriteData(f.cnt, 1))	return FALSE;
	if (!WriteData(f.is_ctl, 1)) return FALSE;
	if (!WriteData(f.speed_h, 1)) return FALSE;
	if (!WriteData(f.speed_l, 1)) return FALSE;
	if (!WriteData(f.angle, 1)) return FALSE;
	if (!WriteData(f.end, 1)) return FALSE;

	return TRUE;
}

void CSerialPort::sendCmd(void)
{
	int cnt = 0x00;
	bool flag = 0;
	Frame f;
	string s;

	while (1)
	{
		mu.lock();
		switch (cmd_id)
		{
		case 0:
		{
			if (cnt_cmd == 0)
				std::cout << "proceed" << std::endl;
			// ǰ��������550���Ƕ�0
			//unsigned char* frame = {"5B", "EB",  cnt, 0x01,  0x02, 0x28, 0x7f};
			f.cnt = cnt;
			f.is_ctl = 0x01;
			f.speed_h = 0x03;
			f.speed_l = 0x84;
			f.angle = 0x70;
			//cmd_id = 9;
			flag = 1;
			cnt_cmd++;
			if (cnt_cmd == 10)
				cmd_id = 9;
			break;
		}
		case 1:
		{
			if (cnt_cmd == 0)
				std::cout << "back" << std::endl;
			// ֹͣ������500���Ƕ�0
			//unsigned char frame[] = { 0x5b,  0xeb,  cnt, 0x01,  0x01, 0xf5, 0x7f };
			f.cnt = cnt;
			f.is_ctl = 0x01;
			f.speed_h = 0x00;
			f.speed_l = 0xc8;
			f.angle = 0x70;
			cnt_cmd++;
			if (cnt_cmd == 6)
				cmd_id = 9;
			flag = 1;
			break;
		}
		case 2:
		{
			if (cnt_cmd == 0)
				std::cout << "turn left" << std::endl;
			// ��ƫ������500����ת45
			//unsigned char frame[] = { 0x5b,  0xeb,  cnt, 0x01,  0x01, 0xf5, 0x00};
			f.cnt = cnt;
			f.is_ctl = 0x01;
			f.speed_h = 0x03;
			f.speed_l = 0x84;
			f.angle = 0x00;
			//cmd_id = 9;
			flag = 1;
			cnt_cmd++;
			if (cnt_cmd == 4)
				cmd_id = 9;
			break;
		}
		case 3:
		{
			if (cnt_cmd == 0)
				std::cout << "turn right" << std::endl;
			// ��ƫ������500����ת45
			//unsigned char frame[] = { 0x5b,  0xeb,  cnt, 0x01,  0x01, 0xf5, 0xff };
			f.cnt = cnt;
			f.is_ctl = 0x01;
			f.speed_h = 0x03;
			f.speed_l = 0x84;
			f.angle = 0xff;
			//cmd_id = 9;
			flag = 1;
			cnt_cmd++;
			if (cnt_cmd == 4)
				cmd_id = 9;
			break;
		}
		case 9:
		{
			//std::cout << "no command" << std::endl;
			f.cnt = cnt;
			f.is_ctl = 0x00;
			f.speed_h = 0x01;
			f.speed_l = 0x28;
			f.angle = 0x70;
			//cmd_id = 9;
			flag = 1;
			break;
		}
		}
		if (cnt == 0x99)
			cnt = 0x00;
		else
			cnt++;

		if (flag)
			WriteFrame(f);

		mu.unlock();

		Sleep(500);
	}
}

void CSerialPort::getCmd()
{
	char cmd = '\n';
	int cmd_tmp = 9;
	while (1)
	{
		cmd = '\n';
		cmd_tmp = 9;
		ifstream f(cmd_file);
		if (!f)
			cout << "failed to open the file!" << endl;
		while (f)
		{
			f >> cmd;
			if (cmd != '\n')
				break;
		}
		cmd_tmp = cmd - '0';
		f.close();
		// ��⵽����
		if (cmd_tmp < 4 && cmd_tmp >= 0)
		{
			//cout << "receive cmd" << endl;
			// �޸ķ��͵�����
			mu.lock();
			cmd_id = cmd_tmp;
			cnt_cmd = 0;
			mu.unlock();

			// ɾ����������
			vector<string> contents;
			string cont_line;
			ifstream fin(cmd_file);
			while (getline(fin, cont_line))
			{
				if (cont_line != "")
					contents.push_back(cont_line);
			}
			fin.close();
			contents.erase(contents.begin());
			contents.erase(contents.begin());
			ofstream fout(cmd_file);
			vector<string>::const_iterator iter = contents.begin();
			for (; iter != contents.end(); iter++)
			{
				fout.write((*iter).c_str(), (*iter).size());
				fout << '\n';
			}
			fout.close();
		}
		Sleep(1000);
	}
}

bool CSerialPort::startThread()
{
	sendCmdThread = std::thread(&CSerialPort::sendCmd, this);
	Sleep(1000);
	getCmdThread = std::thread(&CSerialPort::getCmd, this);

	return TRUE;
}
