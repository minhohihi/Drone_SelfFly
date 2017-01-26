//
// 2015-11-28, jjuiddong
//
// Session ����ü
//
//
// 2016-02-06
//		- gcc �� �۾�
//
#pragma once

#include "network.h"


namespace network
{

	namespace SESSION_STATE {
		enum TYPE {
			DISCONNECT,
			LOGIN_WAIT,
			LOGIN,					// ������ �� ����
			LOGOUT_WAIT,	// ���� ��� ��Ͽ� �ִ� ����
		};
	}


	struct sSession
	{
		SESSION_STATE::TYPE state;
		SOCKET socket;
	};

}
