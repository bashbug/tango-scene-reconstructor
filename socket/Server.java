import java.lang.*;
import java.io.*;
import java.net.*;

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
         Socket socket = server.accept();
         System.out.print("Server has connected!\n");

         InputStream is = socket.getInputStream();
         OutputStream os = new FileOutputStream("out.txt");

         // recieved file data
         copy(is, os);

         is.close();
         os.close();

      } catch(Exception e) {
         System.out.println("Can't establish socket on port: " + port);
      }
   }
}
