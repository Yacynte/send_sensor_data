void sendData() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(UDP_IP);

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection to server failed");
        close(sock);
        return;
    }

    while (true) {
        std::unique_lock<std::mutex> lock(imageMutex);
        imageCondVar.wait(lock, [] { return !imageQueue.empty() || stopImageSaving; });

        if (stopImageSaving && imageQueue.empty()) break;

        auto dataPair = std::move(imageQueue.front());
        imageQueue.pop();
        lock.unlock();

        auto& [image, socket] = dataPair.first;
        auto& [timestamp, label] = dataPair.second;

        // Encode image asynchronously
        auto encodeFuture = std::async(std::launch::async, [&]() {
            std::vector<uchar> buffer;
            cv::imencode(".jpg", image, buffer); // Faster than PNG
            return buffer;
        });

        std::vector<uchar> buffer = encodeFuture.get();

        std::string header = label + "|" + timestamp + "|";
        size_t header_size = header.size();
        size_t image_size = buffer.size();

        // Combine data into a single buffer for fewer network calls
        std::vector<uchar> packet(header.begin(), header.end());
        packet.insert(packet.end(), buffer.begin(), buffer.end());

        if (send(sock, packet.data(), packet.size(), 0) < 0) {
            std::cerr << "Send failed!" << std::endl;
        }
    }

    close(sock);
}
