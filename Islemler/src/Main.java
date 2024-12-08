import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        Scanner scan= new Scanner(System.in);

        //İki Sayı ile 4 Aritmetik İşlem
        int sayiBir = scan.nextInt();
        int sayiIki = scan.nextInt();
        System.out.println(sayiBir+sayiIki);
        System.out.println(sayiBir-sayiIki);
        System.out.println(sayiBir/sayiIki);
        System.out.println(sayiBir*sayiIki);

        //Bir Eşkenar üçgenin alanı ve çevresi
        int yukseklik = scan.nextInt();
        int taban = scan.nextInt();
        System.out.println((yukseklik*taban)/2);
        System.out.println(taban*3);

        //Dairenin Alanı ve Çapı
        int r = scan.nextInt();
        System.out.println(r*r*Math.PI);
        System.out.println(r*2*Math.PI);
    }
}