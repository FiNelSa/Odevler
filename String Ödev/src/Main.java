import java.util.Scanner;
public class Main {
    public static void main(String[] args) {

        Scanner scan = new Scanner(System.in);

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

       System.out.println("Şifreniz");
       String str = scan.nextLine();
        str = str.replace("a" , "@");
        str = str.replace("b" , "5");
        str = str.replace("c" , "*");
        str = str.replace("d" , ">");
        str = str.replace("e" , "é");
        str = str.replace("f" , "3");
        str = str.replace("g" , "6");
        str = str.replace("h" , "4");
        str = str.replace("i" , "!");
        str = str.replace("j" , "1");
        str = str.replace("k" , ";");
        str = str.replace("l" , "|");
        str = str.replace("m" , "3");
        str = str.replace("n" , "-");
        str = str.replace("o" , "0");
        str = str.replace("p" , "<");
        str = str.replace("q" , "%");
        str = str.replace("r" , "/");
        str = str.replace("s" , "2");
        str = str.replace("t" , "+");
        str = str.replace("u" , "&");
        str = str.replace("v" , "_");
        str = str.replace("w" , "`");
        str = str.replace("x" , "8");
        str = str.replace("y" , ":");
        str = str.replace("z" , "=");
        System.out.println("Yeni Şifreniz "+str);

    }
}