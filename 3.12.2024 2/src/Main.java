import java.util.Scanner;
import java.util.Random;

public class Main {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);
        Random rand = new Random();


        //1

        for(int i = 0; i < 3; i++){

            System.out.println("Email'inizi giriniz");
            String email = scan.next();
            System.out.println("Şifrenizi giriniz");
            String password = scan.next();

            if (email.endsWith("@gmail.com")){
                if (password.length()>=8){
                    System.out.println("Başarıyla giriş yapılmıştır");
                    break;
                }else {
                    System.out.println("Şifreniz 8 karakterden kısa. \nTekrar deneyiniz.");
                }
            }else {
                System.out.println("Email'iniz '@gmail.com' ile bitmiyor. \nTekrar deneyiniz.");
            }
        }


        //2

        System.out.println("Faktöriyelinin alınmasını istediğiniz sayıyı giriniz");
        int number = scan.nextInt();

        double ii = 0;
        for (int i = 1; i<=number; i++){
            ii *= i;
        }
        System.out.println(ii);


        //3

        int odd = 0;
        int even = 0;

        for (int i = 0; i<50; i++){
            int iii = rand.nextInt(0, 100);
            if (iii%2==0){
                even++;
            }else {
                odd++;
            }
            System.out.print(ii + " ");
        }

        System.out.print("\n");
        System.out.println("Çift sayı sayısı : " + even);
        System.out.println("Tek sayı sayısı : " + odd);

    }
}