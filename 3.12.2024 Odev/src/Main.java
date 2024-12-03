import java.sql.Array;
import java.util.*;


public class Main {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);
        Random rand = new Random();

        //1

        String[] names1 = {"Semih", "Ufuk", "Efe", "Toprak", "Sarp", "Çağan", "İrem", "Ezel", "Burak", "Sipan"};
        ArrayList<String> names = new ArrayList<>(Arrays.asList(names1));

        for (int i = 0; i<10; i++){
            System.out.println(names.get(i).toUpperCase());
        }

        System.out.println("Hangi ismi değiştirmek istiyorsunuz?");
        int change = scan.nextInt()-1;
        System.out.println("Değiştireceğiniz isim : " + names.get(change).toUpperCase());
        System.out.print("Yeni isim : ");
        names.set(change, scan.next());

        for (int i = 0; i< names.size(); i++){
            System.out.println(names.get(i).toUpperCase());
        }

        System.out.println("Hangi ismi çıkarmak istiyosunuz?");
        int delete = scan.nextInt()-1;
        System.out.println("Sileceğiniz isim : " + names.get(delete).toUpperCase());
        names.remove(delete);

        for (int i = 0; i< names.size(); i++){
            System.out.println(names.get(i).toUpperCase());
        }


        //2

        ArrayList<Integer> numbers = new ArrayList<>();

        System.out.println("3 adet değer giriniz");
        int number1 = scan.nextInt();
        numbers.add(number1);
        int number2 = scan.nextInt();
        numbers.add(number2);
        int number3 = scan.nextInt();
        numbers.add(number3);

        for (int i = 1;i < numbers.size(); i++){
            numbers.set(0, numbers.get(0)+numbers.get(i));
        }
        System.out.println("Toplam : " + numbers.get(0));

        int avrg = numbers.get(0)/(numbers.size());
        System.out.println("Ortalama : " + avrg);


        //3

        ArrayList<Integer> numbers2= new ArrayList<>();
        System.out.println("Rastgele 15 sayı : ");
        for(int i = 0; i<15; i++){
            int ii = rand.nextInt(0, 100);
            System.out.println(ii);
            numbers2.add(ii);
        }

        Collections.sort(numbers2);
        System.out.println("Küçükten Büyüğe : " + numbers2);
    }
}