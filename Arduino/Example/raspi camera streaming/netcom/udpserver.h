//
// v.1
// udp server, receive ���
//
// v.2
// close(), waitforsingleobject
// isConnect()
// add variable m_isReceivData
// thread restart bug fix
//
// 2016-02-09
//		- linux�� �۾�
//
#pragma once

#include "network.h"


namespace network
{

	class cUDPServer
	{
	public:
		cUDPServer();
		virtual ~cUDPServer();

		bool Init(const int id, const int port);
		void SetRecvData(const BYTE *buff, const int buffLen);
		int GetRecvData(OUT BYTE *dst, const int maxSize);
		void SetMaxBufferLength(const int length);
		void Close(const bool isWait = false);
		bool IsConnect() const;


		int m_id;
		SOCKET m_socket;
		int m_port;
		bool m_isConnect;
		BYTE *m_buffer;
		int m_bufferLen;
		bool m_isReceiveData; // ��Ŷ�� �޾Ҵٸ� true�� �ȴ�. GetRecvData() �Լ����� �ʱ�ȭ �ȴ�.
		int m_maxBuffLen;

		pthread_t m_handle;
		pthread_mutex_t m_CriticalSection;
		bool m_threadLoop;
		int m_sleepMillis; // default = 10
	};


	inline bool cUDPServer::IsConnect() const { return m_isConnect; }
}
