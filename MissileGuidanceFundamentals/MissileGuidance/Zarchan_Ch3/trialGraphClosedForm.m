vals = readtable("AdjointClosedFormSol.txt");

time = vals.Var1;

MNT_3 = vals.Var2;

MNT_4 = vals.Var3;

MNT_5 = vals.Var4;

MHE_3 = vals.Var5;

MHE_4 = vals.Var6;

MHE_5 = vals.Var7;

figure(1)
plot(time,MNT_3)
grid on
hold on
plot(time,MNT_4)
plot(time,MNT_5)
ylabel("Miss [ft]")
xlabel("t_f/T")
title("Miss Distance: Target Manuever")
legend("N'=3","N'=4","N'=5")
hold off

figure(2)
plot(time,MHE_3)
grid on
hold on
plot(time,MHE_4)
plot(time,MHE_5)
ylabel("Miss [ft]")
xlabel("t_f/T")
title("Miss Distance: Heading Error")
legend("N'=3","N'=4","N'=5")
hold off