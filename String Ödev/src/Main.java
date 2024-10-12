public class Main {
    public static void main(String[] args) {

        //String Senaryo

       String name1 = "Semih";
       String name2 = "Ufuk";
       String name3 = "Efe";

       int lname1 = name1.length();
       int lname2 = name2.length();
       int lname3 = name3.length();

       if (lname1 > lname2){
           if (lname1 > lname3) {
               System.out.println(name1 + " ismi " + name2 + " isminden ve " + name3 + " isminden uzundur.");
           }else {
               System.out.println(name3 + " ismi " + name1 + " isminden, o da " + name2 + " isminden uzundur.");
           }
       }else if (lname2 > lname3){
           System.out.println(name2 + " ismi " + name1 + " isminden ve " + name3 + " isminden uzundur.");
       }else {
           System.out.println(name3 + " ismi " + name2 + " isminden, o da " + name1 + " isminden uzundur.");
       }

    }
}