
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>


#define INVALID_SOCKET  (SOCKET)(~0)
#define SOCKET_ERROR            (-1)

typedef int SOCKET;

using namespace std;

int main(int argc, char **argv)
{
	sockaddr_in sockAddr;
	//string ip = "127.0.0.1";
	string ip = "169.254.172.62";
	int port = 8888;

	// UDP/IP ��Ʈ�� ������ �����մϴ�.
	// socket(�ּ� �迭, ���� ����, ��������
	SOCKET ssock = socket(AF_INET, SOCK_DGRAM, 0);
	if (ssock == INVALID_SOCKET)
	{
		//clog::Error( clog::ERROR_CRITICAL, "socket() error\n" );
		return false;
	}

	// �ּ� ����ü�� ä��ϴ�.
	bzero(&sockAddr, sizeof(sockaddr_in));
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = inet_addr(ip.c_str());
	sockAddr.sin_port = htons(port);
	int clen = sizeof(sockaddr_in);

	// ������ �����մϴ�
	// connect(����, ���� �ּ�, ���� �ּ��� ����
	int nRet = connect(ssock, (struct sockaddr*)&sockAddr, clen);
	if (nRet == SOCKET_ERROR)
	{
		//clog::Error( clog::ERROR_CRITICAL, "connect() error ip=%s, port=%d\n", ip.c_str(), port );
		close(ssock);
		return false;
	}

	//out = ssock;
	while (1)
	{
		const int slen = sizeof(sockAddr);
		sendto(ssock, "hello", 6, 0, (struct sockaddr*) &sockAddr, slen);
		sleep(1);
	}

	return 0;
}
