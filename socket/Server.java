import java.lang.*;
import java.io.*;
import java.net.*;

import java.text.SimpleDateFormat;
import java.util.Date;

class Server {

   static void copy(InputStream in, OutputStream out) throws IOException {
      byte[] buf = new byte[8192];
      int len = 0;
      while ((len = in.read(buf)) != -1) {
         out.write(buf, 0, len);
      }
   }

   public static void main(String args[]) {
      int port = 2000;

      // check if a port number was passed as command line argument
      if (args.length == 1) {
         try {
            port = Integer.parseInt(args[0]);
         } catch (NumberFormatException e) {
            System.err.println("argument port" + args[0] + " must be an integer.");
            System.exit(1);
         }
      } else {
         System.err.println("usage ./Server port");
         System.exit(1);
      }

      // establish socket on given port
      try {
         byte[] byteArray = new byte[400000];
         ServerSocket server = new ServerSocket(port);
         int counter = 1;

         while(true) {
            Socket socket = server.accept();
            System.out.print("Server has connected!\n");

            InputStream isClient = socket.getInputStream();
            OutputStream osClient = socket.getOutputStream();

            Date date = new Date() ;
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd-hhmmss") ;
            OutputStream osServer = new FileOutputStream("pcd-" + dateFormat.format(date) + "-" + Integer.toString(counter) + ".PCD");

/*            String ans = "Connection established!";
            osClient.write(ans.getBytes("UTF-8"));


            ans = "File written!";
            osClient.write(ans.getBytes("UTF-8"));*/

            // recieved file data
            copy(isClient, osServer);
            counter++;

            isClient.close();
            osClient.close();
            osServer.close();
         }

      } catch(Exception e) {
         System.out.println("Can't establish socket on port: " + port);
      }
   }
}
