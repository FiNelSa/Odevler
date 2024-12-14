import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);
        CAR amgGt = new CAR("MERCEDES", "AMG GT", "gray", 2024, 600, 3501);

        System.out.println("Arabanın markası : " + amgGt.getBrand());
        System.out.println("Arabanın modeli : " + amgGt.getModel());
        System.out.println("Arabanın rengi : " + amgGt.getColor());
        System.out.println("Arabanın motorunun gücü : " + amgGt.getHp());
        System.out.println("Arabanın motorunun hacmi : " + amgGt.getCc());

        System.out.println("Do you want to add a car?");
        boolean i = scan.nextBoolean();
        if (i){
            System.out.println("Arabanın markasını giriniz?");
            String brand = scan.next();
            System.out.println("Arabanın modelini giriniz");
            String model = scan.next();
            System.out.println("Arabanın rengini giriniz");
            String color = scan.next();
            System.out.println("Arabanın yılını giriniz");
            int year = scan.nextInt();
            System.out.println("Arabanın motorunun gücünü giriniz");
            int hp = scan.nextInt();
            System.out.println("Arabanın motorunun hacmini giriniz");
            int cc = scan.nextInt();

            CAR userCar = new CAR(brand, model, color, year, hp, cc);
            System.out.println("Arabanın markası : " + userCar.getBrand());
            System.out.println("Arabanın modeli : " + userCar.getModel());
            System.out.println("Arabanın rengi : " + userCar.getColor());
            System.out.println("Arabanın motorunun gücü : " + userCar.getHp());
            System.out.println("Arabanın motorunun hacmi : " + userCar.getCc());
        }
    }
}