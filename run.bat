@echo on
REM ##################
REM # run experiment #
REM ##################
java -classpath .\classes;.\lib\burlap-3.0.1-jar-with-dependencies.jar  edu.gt.ml.proj4.MyBasicBehavior

REM ############################
REM # run QLearning Experiment #
REM ############################
java -classpath .\classes;.\lib\burlap-3.0.1-jar-with-dependencies.jar  edu.gt.ml.proj4.MyQLearning

REM #####################
REM # run small big MDP #
REM #####################
java -classpath .\classes;.\lib\burlap-3.0.1-jar-with-dependencies.jar  edu.gt.ml.proj4.MySmallBigGrid