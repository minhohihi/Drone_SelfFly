//
// ��Ʈ���ֿ� ���õ� ������ �����Ѵ�.
//
#pragma once


namespace cvproc
{
	// �̹��� ���� ��������
	struct sStreamingData
	{
		BYTE id;				// streaming id (���� ���̵𳢸� ����Ÿ�� ��ģ �Ŀ� ����Ѵ�.)
		BYTE chunkSize;	// chunk size (�̹��� �뷮 ������ ������ ����Ÿ ûũ�� ����)
		BYTE chunkIndex;	// chunk index
		BYTE isGray;		// gray 0 : 1
		BYTE isCompressed;	// jpeg compressed 0 : 1
		int imageBytes;			// image size (byte unit)
		BYTE *data;				// image data
	};


	// ���ù��� �������� ���� ��� ��������
	// TCP/IP �θ� ���۵ȴ�.
	struct sStreamingProtocol
	{
		BYTE protocol;		// �������� Ÿ��
									// protocol = 100, ���� �������� ���� (TCP/IP) 
									// Receiver�� IP�� Port, ���� Ÿ�� �۽�
									// 
									// protocol = 101, gray, compressed, compressed quality ����
									// gray, compressed, compQuality �� ���.
									//


		BYTE type;			// udp=1, tcp=0
		unsigned int uip;
		int port;
		bool gray;
		bool compressed;
		int compQuality;
		int fps;
	};


	const static int g_maxStreamSize = (int)pow(2, 15) - 1;
}
