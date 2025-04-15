package frc.robot.utility;

public class ReefPole {

    private String poleLetter;
    private double levelOne;
    private double levelTwo;
    private double levelThree;
    private double levelFour;
    
    public ReefPole(String poleLetter, double levelOneHeight, double levelTwoHeight, double levelThreeHeight, double levelFourHeight) {
        this.poleLetter = poleLetter;
        this.levelOne = levelFourHeight;
        this.levelTwo = levelTwoHeight;
        this.levelThree = levelThreeHeight;
        this.levelFour = levelFourHeight;
    }

    public String getPoleLetter() {
        return this.poleLetter;
    }

    public double getLevelOne() {
        return this.levelOne;
    }

    public double getLevelTwo() {
        return this.levelTwo;
    }

    public double getLevelThree() {
        return this.levelThree;
    }

    public double getLevelFour() {
        return this.levelFour;
    }
}
