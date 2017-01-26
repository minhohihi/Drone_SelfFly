//
// �̹��� ������ UDP�� ���� �����Ѵ�.
// cStreamingReceiver�� ���� �����Ѵ�.
// �̹����� �����ϰų�, ���� �״�� ������ �� �ִ�.
// �̹��� �뷮�� ũ��, ������ �����Ѵ�.
//
// Protocol
// - (byte) id (�̹����� ��Ÿ���� id)
//	- (byte) chunk size (�̹��� �뷮 ������ ������ ����Ÿ ûũ�� ����)
//	- (byte) chunk index
//	- (byte) gray 0 : 1
//	- (byte) compressed 0 : 1
//	- (int) image size
//
// 2016-02-09
//		- linux �۾�
//		- ó�� ������ tcp/ip�� �̷�����. �� ��, udp�� ��������� ���δ� 
//		sStreamingProtocol ��Ŷ�� Ŭ���̾�Ʈ�� �����ϸ鼭 �����Ѵ�.
//
#pragma once

#include "../netcom/network.h"
#include "opencvcom.h"
#include "streaming.h"


namespace cvproc
{
	class cStreamingSender : public network::iSessionListener
	{
	public:
		cStreamingSender();
		virtual ~cStreamingSender();

		bool Init(const int port, const bool isConvertGray = true, const bool isCompressed = true, const int jpgQuality = 40);
		void CheckPacket();
		int Send(const cv::Mat &image);
		bool IsConnect(const bool isUdp);
		void Close();


	protected:
		int SendImage(const BYTE *data, const int len, const bool isGray, const bool isCompressed);
		bool SendSplit();
		// session event
		virtual void RemoveSession(const SOCKET remoteSock) override;
		virtual void AddSession(const SOCKET remoteSock) override;


	public:
		network::cTCPServer m_tcpServer;

		enum {MAX_UDP_COUNT=2};
		map<SOCKET,int> m_users; // socket = udp index matching, default = -1
		network::cUDPClient m_udpClient[MAX_UDP_COUNT]; // �ִ� ������ ���� ��
		bool m_udpUsed[MAX_UDP_COUNT];							// m_udpClient[] �� ���Ǵ� ���̸�,  true�� ����.

		bool m_isConvertGray;
		bool m_isCompressed;
		int m_jpgCompressQuality;
		int m_fps;
		int m_deltaTime;
		int m_lastSendTime;
		cv::vector<uchar> m_compBuffer;
		cv::Mat m_gray;
		BYTE *m_sndBuffer;


	protected:
		enum STATE {
			NOTCONNECT,	// ������ ���ӵǾ� ���� ���� ����
			READY,	// �̹��� ������ ������ ����
			SPLIT,	// �̹����� �������� ûũ�� ����� ������ ����
		};
		STATE m_state;
		bool m_isUDP;	// udp/ip or tcp/ip

		// Send Split Parameters
		BYTE *m_imagePtr;
		int m_chunkId;
		int m_sendChunkIndex;
		int m_chunkCount;
		int m_imageBytes;
		int m_sendBytes;
	};
}
