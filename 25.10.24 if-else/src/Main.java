import java.util.Random;
import java.util.Scanner;
import java.util.ArrayList;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        Scanner scan = new Scanner(System.in);
        Random sayi = new Random();

        int toplamYatirilanPara = 0;
        int toplamKazanilanPara = 0;
        int kazanilanPara = 0;
        int yatirilanPara = 0;
        int para = 0;
        //Daha sonra üstüne yazabilmek ve paramızın save alınması için önceden while döngüsünün dışında değer veriyoruz.

        while(true) {

            yatirilanPara = scan.nextInt();
            //Kaç para yatıracağımızı alıyor

            System.out.println("Yatırdığınız Para = " + yatirilanPara);
            //Kullanıcıya Bildiriyor

            int[] olasilik = {1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3};
            //Sayıların gelme olasılıği 1 en fazla 7 en az

            int birinci = sayi.nextInt(0, olasilik.length);
            int ikinci = sayi.nextInt(0, olasilik.length);
            int ucuncu = sayi.nextInt(0, olasilik.length);
            //Bu sayılardan birinin sıra numarasını alıyor

//            int birbir = olasilik[birinci];
//            int ikiki = olasilik[ikinci];
//            int ucuc = olasilik[ucuncu];

            System.out.println("\n");
            for (int bir = 0; bir <= olasilik[birinci]; bir++) {
                System.out.print("\r    Birinci Sayı = " + bir);  // Aynı satırı günceller
                Thread.sleep(1000);  // 0.1 saniye bekler
            }//Animasyonlu bir şekilde artması için
            System.out.println("\n");

            for (int iki = 0; iki <= olasilik[ikinci]; iki++) {
                System.out.print("\r    İkinci Sayı = " + iki);  // Aynı satırı günceller
                Thread.sleep(1000);  // 0.1 saniye bekler
            }//Animasyonlu bir şekilde artması için
            System.out.println("\n");

            for (int uc = 0; uc <= olasilik[ucuncu]; uc++) {
                System.out.print("\r    Üçüncü Sayı = " + uc);  // Aynı satırı günceller
                Thread.sleep(1000);  // 0.1 saniye bekler
            }//Animasyonlu bir şekilde artması için
            System.out.println("\n");

            if (birinci == ikinci && birinci == ucuncu) {
                if (olasilik[birinci] == 1) {
                    kazanilanPara = yatirilanPara * 3;
                } else if (olasilik[birinci] == 2) {
                    kazanilanPara = yatirilanPara * 4;
                } else if (olasilik[birinci] == 3) {
                    kazanilanPara = yatirilanPara * 5;
                } else if (olasilik[birinci] == 4) {
                    kazanilanPara = yatirilanPara * 6;
                } else if (olasilik[birinci] == 5) {
                    kazanilanPara = yatirilanPara * 10;
                } else if (olasilik[birinci] == 6) {
                    kazanilanPara = yatirilanPara * 20;
                } else if (olasilik[birinci] == 7) {
                    kazanilanPara = yatirilanPara * 50;
                }//Kaç gelirse kaç para alcağı
            }//olasilik[birinci] yazmamın sebebi sıranın hangi sayıya denk geldiğini öğrenmek

            toplamYatirilanPara +=yatirilanPara;
            toplamKazanilanPara += kazanilanPara;
            //paranın save alınması için
            para = toplamKazanilanPara - toplamYatirilanPara + 1000;
            //paray hesaplama

            System.out.println(para);
        }
    }
}