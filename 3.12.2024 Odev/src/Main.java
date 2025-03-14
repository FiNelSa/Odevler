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

        for (String name : names) {
            System.out.println(name.toUpperCase());
        }

        System.out.println("Hangi ismi çıkarmak istiyosunuz?");
        int delete = scan.nextInt()-1;
        System.out.println("Sileceğiniz isim : " + names.get(delete).toUpperCase());
        names.remove(delete);

        for (String name : names) {
            System.out.println(name.toUpperCase());
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
            numbers.set(0, numbers.getFirst()+numbers.get(i));
        }
        System.out.println("Toplam : " + numbers.getFirst());

        int avrg = numbers.getFirst()/(numbers.size());
        System.out.println("Ortalama : " + avrg);


        //3

        ArrayList<Integer> numbers2= new ArrayList<>();
        System.out.println("Rastgele 15 sayı : ");

        for(int i = 0; i<15; i++){
            int ii = rand.nextInt(0, 100);
            numbers2.add(ii);
        }
        System.out.println(numbers2);

        int index = 0;
        while (index < numbers2.size()){
            if (index == 0)
                index++;
            if (numbers2.get(index)>=numbers2.get(index-1))
                index++;
            else{
                int temp = numbers2.get(index);
                numbers2.set(index, numbers2.get(index-1));
                numbers2.set(index-1, temp);
                index--;
            }
        }

        System.out.println("\nKüçükten Büyüğe : \n" + numbers2);
    }
}