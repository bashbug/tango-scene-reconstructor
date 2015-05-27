import java.lang.*;
import java.io.*;
import java.net.*;
import java.nio.file.*;
import static java.nio.file.StandardWatchEventKinds.*;
import static java.nio.file.LinkOption.*;
import java.util.*;

class Client {

   static void copy(InputStream in, OutputStream out) throws IOException {
      byte[] buf = new byte[8192];
      int len = 0;
      while ((len = in.read(buf)) != -1) {
         out.write(buf, 0, len);
      }
   }

   private final WatchService watcher;
   private final Map<WatchKey,Path> keys;
   private boolean trace = false;

   @SuppressWarnings("unchecked")
   static <T> WatchEvent<T> cast(WatchEvent<?> event) {
      return (WatchEvent<T>)event;
   }

   private void register(Path dir) throws IOException {
      WatchKey key = dir.register(watcher, ENTRY_CREATE, ENTRY_DELETE, ENTRY_MODIFY);
      if (trace) {
         Path prev = keys.get(key);
         if (prev == null) {
            System.out.format("register: %s\n", dir);
         } else {
            if (!dir.equals(prev)) {
               System.out.format("update: %s -> %s\n", prev, dir);
            }
         }
      }
      keys.put(key, dir);
   }

   Client(Path dir) throws IOException {
     this.watcher = FileSystems.getDefault().newWatchService();
     this.keys = new HashMap<WatchKey,Path>();
     register(dir);
     // enable trace after initial registration
     this.trace = true;
   }

   private String processEvents() {
      int counter = 0;
      while (true) {
         counter++;
         // wait for key to be signalled
         WatchKey key = null;
         Path fileName = null;

         try {
            key = watcher.take();
         } catch (InterruptedException x) {
            System.exit(1);
         }

         Path dir = keys.get(key);
         if (dir == null) {
            System.err.println("WatchKey not recognized!!");
            continue;
         }
         boolean create = false;
         boolean modify = false;
         boolean delete = false;
         for (WatchEvent<?> event: key.pollEvents()) {
            WatchEvent.Kind kind = event.kind();

            // TBD - provide example of how OVERFLOW event is handled
            if (kind == OVERFLOW) {
              continue;
            }

            // Context for directory entry event is the file name of entry
            WatchEvent<Path> ev = cast(event);
            fileName = ev.context();
            Path child = dir.resolve(fileName);

            if (kind == ENTRY_CREATE) {
               System.out.println(counter + ": CREATE");
               create = true;
            }
            if (kind == ENTRY_MODIFY) {
               System.out.println(counter + ": MODIFY");
               modify = true;
            }
            if (kind == ENTRY_DELETE) {
               System.out.println(counter + ": DELETE");
               delete = true;
            }
         }

         if (create && delete) {
            System.out.println("New FILE");
            return fileName.toString();
         }

         // reset key and remove from set if directory no longer accessible
         boolean valid = key.reset();
         if (!valid) {
             keys.remove(key);
            // all directories are inaccessible
            if (keys.isEmpty()) {
              break;
            }
         }
      }
      return "-1";
   }

   public static void main(String args[]) {
      String hostname = "localhost";
      int port = 2000;


      if (args.length == 2) {
         hostname = args[0];
         // check if a port number was passed as command line argument
         try {
           port = Integer.parseInt(args[1]);
         } catch (NumberFormatException e) {
           System.err.println("argument port" + args[1] + " must be an integer.");
           System.exit(1);
         }
      } else {
         System.err.println("usage ./Server ip port file");
         System.exit(1);
      }

      try {
         // client socket
         Socket socket = new Socket(hostname, port);
         // register directory and process its events
         Path dir = Paths.get("files/");
         Client watchForNewFiles = new Client(dir);
         String newFileName = watchForNewFiles.processEvents();
         System.out.println(newFileName);

         System.out.println("filepath: " + dir +newFileName);
         File file = new File(dir + "/" + newFileName);
         // read file into buffer
         byte[] byteArray = new byte[(int) file.length()];
         InputStream is = new FileInputStream(file);
         OutputStream os = socket.getOutputStream();

         // send file to server via socket
         copy(is, os);

         is.close();
         os.close();

      } catch(Exception e) {
         System.out.print("Whoops! It didn't work!\n");
      }
   }
}
