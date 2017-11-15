#include <iostream>
#include <signal.h>
#include <fstream>
#include <cstdlib>
#include <chrono>

#include <asio.hpp>
#include <turbojpeg.h>
#include "KinectSDK2Streamer.hpp"

asio::io_service io_service_;

int main(int argc, char *argv[]) {
	oi::core::rgbd::StreamerConfig cfg;
	cfg.Parse(argc, argv);
	std::cout << "CFG: " << cfg.listenPort << " <=> " << cfg.remoteHost << ":" << cfg.remotePort << std::endl;

	if (cfg.useMatchMaking) {
		oi::core::network::UDPConnector client(cfg.listenPort, cfg.remotePort, cfg.remoteHost, io_service_);
		client.Init(cfg.socketID, cfg.deviceSerial, true, 1024, 32, 65506, 64);

		oi::core::rgbd::KinectDSK2Streamer k2stream(cfg, &client);
		k2stream.Run();
		k2stream.Exit();
	} else {
		oi::core::network::UDPBase client(cfg.listenPort, cfg.remotePort, cfg.remoteHost, io_service_);
		client.Init(1024, 32, 65506, 32);

		oi::core::rgbd::KinectDSK2Streamer k2stream(cfg, &client);
		k2stream.Run();
		k2stream.Exit();
	}

	system("pause");
}
