//
// 2015-11-28, jjuiddong
//
// TCP/IP ���������� �̿��ؼ� ����ϴ� ��ü��.
// �ִ��� �����ϰ� �������.
//
// cTCPClient �� ��Ŷ�� ��ȯ�ϰ� ����Ǿ���.
//
// 2016-02-06
//		- gcc �� �۾�
//
#pragma once

#include "network.h"


namespace network
{

	class iSessionListener
	{
	public:
		virtual void RemoveSession(const SOCKET remoteSock) = 0;
		virtual void AddSession(const SOCKET remoteSock) = 0;
	};


	class cTCPServer
	{
	public:
		cTCPServer();
		virtual ~cTCPServer();

		bool Init(const int port, const int packetSize = 512, const int maxPacketCount = 10, const int sleepMillis = 30);
		void SetListener(iSessionListener *listener);
		void Close();
		bool IsConnect() const;
		int MakeFdSet(OUT fd_array &out);
		bool AddSession(const SOCKET remoteSock);
		void RemoveSession(const SOCKET remoteSock);


		SOCKET m_svrSocket;
		vector<sSession> m_sessions;
		iSessionListener *m_listener;
		int m_port;
		bool m_isConnect;
		int m_maxBuffLen;
  		cPacketQueue m_sendQueue;
  		cPacketQueue m_recvQueue;

		pthread_t m_handle;
 		pthread_mutex_t m_criticalSection;
		bool m_threadLoop;
		int m_sleepMillis;
	};


	inline bool cTCPServer::IsConnect() const { return m_isConnect; }
}
