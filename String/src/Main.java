import java.util.Locale;
import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);
        String sifre = scan.nextLine();
        sifre = sifre.toLowerCase();
        sifre = sifre.replace("a" ,"@");
        sifre = sifre.replace("b" ,">");
        sifre = sifre.replace("c" ,"*");
        sifre = sifre.replace("d" ,"<");
        sifre = sifre.replace("e" ,"-");
        sifre = sifre.replace("f" ,"=");
        sifre = sifre.replace("g" ,"6");
        sifre = sifre.replace("h" ,"4");
        sifre = sifre.replace("i" ,"1");
        sifre = sifre.replace("j" ,"7");
        sifre = sifre.replace("k" ,";");
        sifre = sifre.replace("l" ,"|");
        sifre = sifre.replace("m" ,"3");
        sifre = sifre.replace("n" ," ");
        sifre = sifre.replace("o" ,"0");
        sifre = sifre.replace("p" ,"?");
        sifre = sifre.replace("q" ,"_");
        sifre = sifre.replace("r" ,"`");
        sifre = sifre.replace("s" ,"5");
        sifre = sifre.replace("t" ,"+");
        sifre = sifre.replace("u" ,"/");
        sifre = sifre.replace("v" ,".");
        sifre = sifre.replace("w" ,":");
        sifre = sifre.replace("x" ,"é");
        sifre = sifre.replace("y" ,"æ");
        sifre = sifre.replace("z" ,"&");

        System.out.println(sifre);
    }
}