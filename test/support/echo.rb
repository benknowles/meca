require 'socket'

Socket.tcp_server_loop(10000) do |conn, addr|
  Thread.new do
    client = "#{addr.ip_address}:#{addr.ip_port}"
    puts "#{client} is connected"

    conn.print("[3000][Connected to Meca 500 x_x_x.x.x.]\0")

    begin
      loop do
        line = conn.readline("\0").strip
        puts "#{client} says: #{line}"


        if line.include?("GetJoints")
          conn.print("[2026][1.0,2.0,3.0,4.0,5.0,6.0]\0")
        else
          conn.print(line + "\0")
        end
      end
    rescue EOFError
      conn.close
      puts "#{client} has disconnected"
    end
  end
end

