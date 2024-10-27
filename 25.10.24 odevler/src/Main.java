import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);

        //Sınav Ortalama Alma
        System.out.println("Vize sınavı notunuz?");
        int vizeSinav = scan.nextInt();
        System.out.println("Final sınavı notunuz?");
        int finalSinav = scan.nextInt();
        //Değerleri Alıyoruz

        double ortalama = (vizeSinav * 0.35) + (finalSinav * 0.65);
        //Ne kadar etkilediklerine bağlı olarak çarpıp topluyoruz
        if (75 <= ortalama) {
            //Karşılaştırma
            System.out.println(ortalama + " ortalama ile geçtiniz.");
        } else {
            //Değilse
            System.out.println(ortalama + " ortalama ile kaldınız.");
        }

        //Sayıları Karşılaştırma
        System.out.println("Karşılatırmak için iki sayı giriniz");
        int birinciSayi = scan.nextInt();
        int ikinciSayi = scan.nextInt();
        //Değerleri Alıyoruz

        if (birinciSayi > ikinciSayi) {
            //Karşılaştırma
            System.out.println(birinciSayi + " " + ikinciSayi + "'dan büyüktür.");
        } else if (ikinciSayi > birinciSayi) {
            //Değilse ve Böyleyse
            System.out.println(ikinciSayi + " " + birinciSayi + "'dan büyüktür.");
        } else {
            //İkisi de değilse
            System.out.println("İki sayı birbirine eşittir.");
        }

        //E-Posta Onay
        System.out.println("E-postanızı giriniz.");
        String ePosta = scan.next();
        String sonOnHane = ePosta.substring(ePosta.length()-10);

        if (ePosta.length()<=10){
            System.out.println("Geçerli bir epota giriniz");
        }else if (sonOnHane.equals("@gmail.com")){
            System.out.println("E-postanız : " + ePosta);
        }else {
            System.out.println("Sonu '@gmail.com' olan bir e-porta kullanınız.");
        }

        //Sayılarla İşlemler
        while (true) {//Sürekli olması için
            System.out.println("Lütfen işlem türünü seçin (+, -, *, /) veya çıkmak için 'escape' yazın:");
            String islemTuru = scan.next();
            //operatörü alıp küçük harflere çeviriyoruz ki büyük harflerle yazan olursa sorun çıkmasın

            if (islemTuru.equals("+")||islemTuru.equals("-")||islemTuru.equals("*")||islemTuru.equals("/")||islemTuru.equals("escape")){
            //İşlem türünün doğru olup olmadığını kontrol eder
            if (islemTuru.equals("escape")) {
            //Eğer "escape" yazılırsa çıması için
                System.out.println("Program sonladırılıyor.");
                break;
            } else {
                System.out.println("Kaç sayı ile işlem yapmak istiyorsunuz?");
                int sayiAdedi = scan.nextInt();
                //Sayı adedini alır
                if (sayiAdedi<2){
                    System.out.println("En az iki sayı ile işlem yapılabilir");
                    break;
                    //Bir sayı varsa işlem iptal edilir
                }else {
                    double[] sayilar = new double[sayiAdedi];
                    //Sayıların değerini alan bir array oluşturur
                    for (int i = 0; i < sayiAdedi; i++) {
                        System.out.print((i + 1) + ". sayıyı girin: ");
                        try {
                            sayilar[i] = scan.nextDouble();
                            //Hepsini teker teker alıp array e ekler
                        } catch (NumberFormatException e) {
                            //Eğer cevap bir sayı değilse tekrar sorar
                            System.out.println("Geçerli bir sayı giriniz!");
                            i--; // Sayıyı tekrar sor
                        }
                    }
                    double sonuc = sayilar[0];
                    if (islemTuru.equals("+")){
                        //İşlem türü + ise
                        for (int a = 1; a < sayiAdedi; a++) {
                            sonuc += sayilar[a];
                            //Hepsini teker teker birinci ile toplar
                        }
                    }else if (islemTuru.equals("-")){
                        //+ değil ve - ise
                        for (int a = 1; a < sayiAdedi; a++) {
                            sonuc -= sayilar[a];
                            //Hepsini teker teker birinci sayıdan çıkarır
                        }
                    }else if (islemTuru.equals("*")){
                        //+ veya - değil ve * ise
                        for (int a = 1; a < sayiAdedi; a++) {
                            sonuc *= sayilar[a];
                            //Hepsini teker teker birinci sayı ile çarpar
                        }
                    }else if (islemTuru.equals("/")){
                        //+, - veya * değil ve / ise
                        for (int a = 1; a < sayiAdedi; a++) {
                            if (sayilar[a]==0){
                                //sayılardan biri 0 ise işlemi iptal eder
                                System.out.println("Bir sayı 0'a bölünemez! İşlem iptal edildi.");
                                sonuc = Double.NaN;
                                break;
                            }
                            sonuc /= sayilar[a];
                            //Hepsini teker teker 1. sayıdan böler
                        }
                    }

                        System.out.println(sonuc);
                    }
                }
            }else{
                System.out.println("Geçersiz operatör. İşlem iptal ediliyor");
                break;
            }
        }
    }
}
