import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);
        CAR amgGt = new CAR("MERCEDES", "AMG GT", "gray", 2024, 600, 3501);

        System.out.println("Arabanın markası : " + amgGt.getBrand());
        System.out.println("Arabanın modeli : " + amgGt.getModel());
        System.out.println("Arabanın rengi : " + amgGt.getColor());
        System.out.println("Arabanın yılı : " + amgGt.getYear());
        System.out.println("Arabanın motorunun gücü : " + amgGt.getHp());
        System.out.println("Arabanın motorunun hacmi : " + amgGt.getCc());

        System.out.println("Do you want to add a car?");
        boolean i = scan.nextBoolean();
        if (i){
            CAR userCar = new CAR();
            System.out.println("Arabanın markasını giriniz?");
            String brand = scan.next();
            userCar.setBrand(brand);
            System.out.println("Arabanın modelini giriniz");
            String model = scan.next();
            userCar.setModel(model);
            System.out.println("Arabanın rengini giriniz");
            String color = scan.next();
            userCar.setColor(color);
            System.out.println("Arabanın yılını giriniz");
            int year = scan.nextInt();
            userCar.setYear(year);
            System.out.println("Arabanın motorunun gücünü giriniz");
            int hp = scan.nextInt();
            userCar.setHp(hp);
            System.out.println("Arabanın motorunun hacmini giriniz");
            int cc = scan.nextInt();
            userCar.setCc(cc);

            System.out.println("Arabanın markası : " + userCar.getBrand());
            System.out.println("Arabanın modeli : " + userCar.getModel());
            System.out.println("Arabanın rengi : " + userCar.getColor());
            System.out.println("Arabanın motorunun gücü : " + userCar.getHp());
            System.out.println("Arabanın motorunun hacmi : " + userCar.getCc());
        }
    }
}