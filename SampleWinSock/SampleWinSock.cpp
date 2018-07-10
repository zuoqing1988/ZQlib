#include "ZQ_WinSockServerSimple.h"
#include "ZQ_WinSockClientSimple.h"
#include <string>
#include <vector>
#include <sstream>
#pragma comment(lib,"ws2_32.lib")

using namespace ZQ;

int server(int argc, char** argv);
int client(int argc, char** argv);

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		printf("%s server args\n", argv[0]);
		printf("%s client args\n", argv[0]);
		return EXIT_FAILURE;
	}

	if (_strcmpi(argv[1], "server") == 0)
	{
		return server(argc, argv);
	}
	else if (_strcmpi(argv[1], "client") == 0)
	{
		return client(argc, argv);
	}
	else
	{
		printf("%s server args\n",argv[0]);
		printf("%s client args\n",argv[0]);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

/*************************************************/
bool _is_blank_c(char c)
{
	return c == ' ' || c == '\t' || c == '\n';
}
std::vector<std::string>  _split_blank(const char* str)
{
	std::vector<std::string> out;
	int len = strlen(str);
	std::vector<char> buf(len + 1);
	int i = 0, j = 0;
	while (1)
	{
		//skip blank
		for (; i < len && _is_blank_c(str[i]); i++);
		if (i >= len)
			break;

		for (j = i; j < len && !_is_blank_c(str[j]); j++);
		int tmp_len = j - i;
		if (tmp_len == 0)
			break;
		memcpy(&buf[0], str + i, tmp_len * sizeof(char));
		buf[tmp_len] = '\0';

		out.push_back(std::string(&buf[0]));
		i = j;
	}
	return out;
}

bool string_to_args(const std::string& msg, int& argc, char**& argv, std::vector<std::string>& buffer, std::vector<char*>& buffer_ptr)
{
	buffer = _split_blank(msg.c_str());
	argc = buffer.size();
	if (argc == 0)
		return false;
	buffer_ptr.resize(argc);
	for (int i = 0; i < buffer.size(); i++)
		buffer_ptr[i] = (char*)(buffer[i].c_str());
	argv = &buffer_ptr[0];
	return true;
}

int server(int argc, char** argv)
{
	USHORT port = 1234;
	if (argc < 3)
	{
		printf("%s %s port\n", argv[0], argv[1]);
		return EXIT_FAILURE;
	}
	port = atoi(argv[2]);
	ZQ_WinSockIOPoolSimple m_IO;
	ZQ_WinSockServerSimple m_server;

	if (!m_server.InitWinsock())
	{
		printf("failed to init win sock!\n");
		return EXIT_FAILURE;
	}
	m_server.BindIOObj(&m_IO);
	if (!m_server.StartListening(port))
	{
		printf("failed to listen port %d\n", port);
		return EXIT_FAILURE;
	}

	int tmp_argc;
	char** tmp_argv;
	std::vector<std::string> buffer;
	std::vector<char*> buffer_ptr;
	std::ostringstream oss;
	SOCKET sock;
	std::string msg;
	while (true)
	{
		if (m_server.ConsumePacket(sock, msg, 10))
		{
			if (!string_to_args(msg, tmp_argc, tmp_argv, buffer, buffer_ptr))
				continue;

			printf("received: %s\n", msg.c_str());

			oss.str("");
			for (int i = 0; i < tmp_argc; i++)
				oss << "argv" << i << ":" << tmp_argv[i] << " ";
			
			m_server.Send(sock, oss.str());
			if (tmp_argc > 1 &&_strcmpi(tmp_argv[1], "shutdown") == 0)
			{
				break;
			}
		}
		else
		{
			Sleep(5);
		}
	}
	m_server.BroadCast("shutdown");
	Sleep(10000);
	m_server.Stop();
	m_server.ShutdownWinsock();
	return EXIT_SUCCESS;
}

int client(int argc, char** argv)
{
	USHORT port = 1234;
	if (argc < 4)
	{
		printf("%s %s addr port\n", argv[0], argv[1]);
		return EXIT_FAILURE;
	}
	const char* addr = argv[2];
	port = atoi(argv[3]);
	ZQ_WinSockIOPoolSimple m_IO;
	ZQ_WinSockClientSimple m_client;

	if (!m_client.InitWinsock())
	{
		printf("failed to init win sock!\n");
		return EXIT_FAILURE;
	}
	m_client.BindIOObj(&m_IO);

	if (!m_client.Connect(addr, port))
	{
		printf("failed to connect to server ip: %s port: %d\n", addr, port);
		return EXIT_FAILURE;
	}

	const static int BUF_LEN = 500;
	char buf[BUF_LEN];
	while (true)
	{
		std::string msg;
		gets_s(buf, BUF_LEN);
		m_client.Send(std::string(buf));
		while (!m_client.ConsumePacket(msg, 1000));
		puts(msg.c_str());
		if (_strcmpi(msg.c_str(), "shutdown") == 0)
			break;
	}

	m_client.Close();
	m_client.ShutdownWinsock();
	return EXIT_SUCCESS;
}