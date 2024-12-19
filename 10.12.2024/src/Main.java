import java.util.ArrayList;
import java.util.Scanner;

public class Main {
    Scanner scan = new Scanner(System.in);
    public void Multiplication(){

        System.out.println("How many numbers do you want to add?");
        int manyNumber = scan.nextInt();

        int[] numbers = new int[manyNumber];

        for (int i = 0; i < manyNumber; i++){
            int ii = i+1;
            System.out.println("Enter the " + ii + ". number");
            numbers[i] = scan.nextInt();
        }

        for (int i = 1; i<manyNumber; i++){
            numbers[0] = numbers[0]*numbers[i];
        }

        System.out.println("The multiplication of the numbers you entered is : " + numbers[0]);
    }

    public void Marks() {
        ArrayList<Integer> markList = new ArrayList<>();

        System.out.println("What is your first mark?");
        int firstMark = scan.nextInt();
        markList.add(firstMark);
        System.out.println("What is your second mark?");
        int secondMark = scan.nextInt();
        markList.add(secondMark);
        System.out.println("What is your third mark?");
        int thirdMark = scan.nextInt();
        markList.add(thirdMark);

        System.out.println(markList);

        int ii = markList.getFirst();
        for (int i = 1; i<markList.size(); i++){
            markList.set(0, markList.getFirst()+markList.get(i));
        }

        int avrg = markList.getFirst()/markList.size();
        System.out.println(avrg);

        markList.set(0, ii);

        System.out.println("Do you want to add more marks? True or false.");
        boolean moreOrOkay = scan.nextBoolean();

        for (int i= 1; i< markList.size(); i++){
            for (int iii = 0; iii < i; iii++){
                if (markList.get(i)< markList.get(iii)){
                    int j = markList.get(i);
                    markList.set(i, markList.get(iii));
                    markList.set(iii, j);
                }
            }
        }
        System.out.println(markList);

        if (moreOrOkay){
            MoreMarks(markList);
        }
    }

    public void MoreMarks(ArrayList<Integer> markList){

        System.out.println("How many marks do you want to add?");
        int moreMarks = scan.nextInt();

        for (int i = 0; i < moreMarks; i++){
            System.out.println("Enter a mark");
            int ii = scan.nextInt();
            markList.add(ii);
        }

        int ii = markList.getFirst();
        for (int i = 1; i<markList.size(); i++){
            markList.set(0, markList.getFirst()+markList.get(i));
        }

        int avrg = markList.getFirst()/markList.size();
        markList.set(0, ii);
        System.out.println(markList);
        System.out.println(avrg);

        System.out.println("highest to lowest : ");

        for (int i= 1; i< markList.size(); i++){
            for (int iii = 0; iii < i; iii++){
                if (markList.get(i)< markList.get(iii)){
                    int j = markList.get(i);
                    markList.set(i, markList.get(iii));
                    markList.set(iii, j);
                }
            }
        }
        System.out.println(markList);
    }

    public void MarkCalc(){

        System.out.println("Enter you vize mark.");
        int Vize  = scan.nextInt();
        System.out.println("Enter your final mark.");
        int Final = scan.nextInt();

        double Mark = (Vize*0.4)+(Final*0.6);
        System.out.println(Mark);
    }
    public static void main(String[] args) {
        Multiplication();
        Marks();
        MarkCalc();
    }
}