% author: Claudio Gaz, Marco Cognetti
% date: August 2, 2019
% 
% -------------------------------------------------
% Parameters Retrieval Algorithm
% -------------------------------------------------
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
% Identification of the Franka Emika Panda Robot With Retrieval of Feasible
% Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
%
% the following code has been tested on Matlab 2018b

% error_fcn_gM_LMI returns the value of the cost function (loss) when 
% phi(p_k) = || pi(p_k) - pi_hat ||^2 is used

function loss = error_fcn_gM_LMI(x)

global SA_step

% set the proper penalty coefficient (that is variable \xi_k in (29))
if SA_step == 1
    penalty = 0;
else
    penalty = 10*(SA_step-1);
end

% get the current values of the parameters
m1 = x(1);
m2 = x(2);
m3 = x(3);
m4 = x(4);
m5 = x(5);
m6 = x(6);
m7 = x(7);
c1x = x(8);
c1y = x(9);
c1z = x(10);
c2x = x(11);
c2y = x(12);
c2z = x(13);
c3x = x(14);
c3y = x(15);
c3z = x(16);
c4x = x(17);
c4y = x(18);
c4z = x(19);
c5x = x(20);
c5y = x(21);
c5z = x(22);
c6x = x(23);
c6y = x(24);
c6z = x(25);
c7x = x(26);
c7y = x(27);
c7z = x(28);
I1xx = x(29);
I1xy = x(30);
I1xz = x(31);
I1yy = x(32);
I1yz = x(33);
I1zz = x(34);
I2xx = x(35);
I2xy = x(36);
I2xz = x(37);
I2yy = x(38);
I2yz = x(39);
I2zz = x(40);
I3xx = x(41);
I3xy = x(42);
I3xz = x(43);
I3yy = x(44);
I3yz = x(45);
I3zz = x(46);
I4xx = x(47);
I4xy = x(48);
I4xz = x(49);
I4yy = x(50);
I4yz = x(51);
I4zz = x(52);
I5xx = x(53);
I5xy = x(54);
I5xz = x(55);
I5yy = x(56);
I5yz = x(57);
I5zz = x(58);
I6xx = x(59);
I6xy = x(60);
I6xz = x(61);
I6yy = x(62);
I6yz = x(63);
I6zz = x(64);
I7xx = x(65);
I7xy = x(66);
I7xz = x(67);
I7yy = x(68);
I7yz = x(69);
I7zz = x(70);

% initialize error vector
e = zeros(41,1);

% compute error vector, as the difference of current dynamic coeff values
% and previously estimated dyn coeff values, as follows:
% pi(p_k) - pi_hat

e(1) = I2yy + I1zz + c1x^2*m1 + c2x^2*m2 + c1y^2*m1 + c2z^2*m2 - 2.921667e-02;
e(2) = I2xx - 1.0*I2yy + I3yy + 0.099856*m3 + 0.10666225*m4 + 0.10666225*m5 + 0.10666225*m6 + 0.10666225*m7 + 0.632*c3z*m3 - 1.0*c2x^2*m2 + c3x^2*m3 + c2y^2*m2 + c3z^2*m3 - 9.818184e-01;
e(3) = I2xy - 1.0*c2x*c2y*m2 - -5.191244e-03;
e(4) = I2xz - 1.0*c2x*c2z*m2 - 2.841995e-02;
e(5) = I2yz - 1.0*c2y*c2z*m2 - -3.493413e-03;
e(6) = I3yy + I2zz + 0.099856*m3 + 0.10666225*m4 + 0.10666225*m5 + 0.10666225*m6 + 0.10666225*m7 + 0.632*c3z*m3 + c2x^2*m2 + c3x^2*m3 + c2y^2*m2 + c3z^2*m3 - 1.042777e+00;
e(7) = I3xx - 1.0*I3yy + I4yy - 0.00680625*m4 - 1.0*c3x^2*m3 + c4x^2*m4 + c3y^2*m3 + c4z^2*m4 - 1.059003e-02;
e(8) = I3xz - 1.0*c3x*c3z*m3 - -1.043855e-02;
e(9) = I3yz - 1.0*c3y*c3z*m3 - -4.845789e-03;
e(10) = I4yy + I3zz + 0.00680625*m4 + 0.0136125*m5 + 0.0136125*m6 + 0.0136125*m7 + c3x^2*m3 + c4x^2*m4 + c3y^2*m3 + c4z^2*m4 - 1.168757e-01;
e(11) = I4xx - 1.0*I4yy + I5yy + 0.14064975*m5 + 0.14064975*m6 + 0.14064975*m7 + 0.768*c5z*m5 - 1.0*c4x^2*m4 + c5x^2*m5 + c4y^2*m4 + c5z^2*m5 - 5.324312e-01;
e(12) = I4xy + 0.03168*m5 + 0.03168*m6 + 0.03168*m7 + 0.0825*c5z*m5 - 1.0*c4x*c4y*m4 - 1.500656e-01;
e(13) = I4xz - 1.0*c4x*c4z*m4 - 4.772233e-03;
e(14) = I4yz - 1.0*c4y*c4z*m4 - -2.731790e-03;
e(15) = I5yy + I4zz + 0.15426225*m5 + 0.15426225*m6 + 0.15426225*m7 + 0.768*c5z*m5 + c4x^2*m4 + c5x^2*m5 + c4y^2*m4 + c5z^2*m5 - 6.369356e-01;
e(16) = I5xx - 1.0*I5yy + I6yy + 0.007744*m7 - 1.0*c5x^2*m5 + c6x^2*m6 + c5y^2*m5 + c6z^2*m6 - 3.215369e-02;
e(17) = I5xy - 1.0*c5x*c5y*m5 - -3.730935e-03;
e(18) = I5xz - 1.0*c5x*c5z*m5 - -6.133119e-03;
e(19) = I5yz - 1.0*c5y*c5z*m5 - 7.715324e-03;
e(20) = I6yy + I5zz + 0.007744*m7 + c5x^2*m5 + c6x^2*m6 + c5y^2*m5 + c6z^2*m6 - 1.806937e-02;
e(21) = I6xx - 1.0*I6yy + I7yy - 0.007744*m7 - 1.0*c6x^2*m6 + c7x^2*m7 + c6y^2*m6 + c7z^2*m7 - -6.531748e-03;
e(22) = I6xy + 0.088*c7z*m7 - 1.0*c6x*c6y*m6 - 5.365261e-03;
e(23) = I6yz - 1.0*c6y*c6z*m6 - 8.204412e-04;
e(24) = I7yy + I6zz + 0.007744*m7 + c6x^2*m6 + c7x^2*m7 + c6y^2*m6 + c7z^2*m7 - 2.504593e-02;
e(25) = I7xx - 1.0*I7yy - 1.0*c7x^2*m7 + c7y^2*m7 - -1.767591e-03;
e(26) = I7xy - 1.0*c7x*c7y*m7 - 1.185322e-03;
e(27) = I7xz - 1.0*c7x*c7z*m7 - 1.757815e-03;
e(28) = I7yz - 1.0*c7y*c7z*m7 - -5.969360e-04;
e(29) = I7zz + c7x^2*m7 + c7y^2*m7 - 1.272440e-03;
e(30) = c2x*m2 - -5.368710e-03;
e(31) = c2y*m2 - 0.316*m4 - 0.316*m5 - 0.316*m6 - 0.316*m7 - 0.316*m3 - 1.0*c3z*m3 - -3.102576e+00;
e(32) = 0.0825*m4 + 0.0825*m5 + 0.0825*m6 + 0.0825*m7 + c3x*m3 - 6.873555e-01;
e(33) = c3y*m3 - 1.0*c4z*m4 - 2.337654e-02;
e(34) = c4x*m4 - 0.0825*m6 - 0.0825*m7 - 0.0825*m5 - -4.888364e-01;
e(35) = 0.384*m5 + 0.384*m6 + 0.384*m7 + c4y*m4 + c5z*m5 - 1.718486e+00;
e(36) = c5x*m5 - -1.000420e-02;
e(37) = c5y*m5 - 1.0*c6z*m6 - 7.705313e-02;
e(38) = 0.088*m7 + c6x*m6 - 1.656804e-01;
e(39) = c6y*m6 - 1.0*c7z*m7 - -6.771177e-02;
e(40) = c7x*m7 - 6.231087e-03;
e(41) = c7y*m7 - -4.880859e-04;

loss = e'*e;

%------------------------------------------------
% External Penalties
%------------------------------------------------

% conditions on total mass

min_mass = 16;
max_mass = 20;

if m1+m2+m3+m4+m5+m6+m7 < min_mass
    loss = loss + penalty*(min_mass-(m1+m2+m3+m4+m5+m6+m7));
end
if m1+m2+m3+m4+m5+m6+m7 > max_mass
    loss = loss + penalty*(m1+m2+m3+m4+m5+m6+m7-max_mass);
end

% conditions on inertia tensors: triangle inequalities

% link 1
I1 = [I1xx,I1xy,I1xz ; I1xy,I1yy,I1yz ; I1xz,I1yz,I1zz];
loss = check_inertia_condition(I1,loss,penalty);
% link 2
I2 = [I2xx,I2xy,I2xz ; I2xy,I2yy,I2yz ; I2xz,I2yz,I2zz];
loss = check_inertia_condition(I2,loss,penalty);
% link 3
I3 = [I3xx,I3xy,I3xz ; I3xy,I3yy,I3yz ; I3xz,I3yz,I3zz];
loss = check_inertia_condition(I3,loss,penalty);
% link 4
I4 = [I4xx,I4xy,I4xz ; I4xy,I4yy,I4yz ; I4xz,I4yz,I4zz];
loss = check_inertia_condition(I4,loss,penalty);
% link 5
I5 = [I5xx,I5xy,I5xz ; I5xy,I5yy,I5yz ; I5xz,I5yz,I5zz];
loss = check_inertia_condition(I5,loss,penalty);
% link 6
I6 = [I6xx,I6xy,I6xz ; I6xy,I6yy,I6yz ; I6xz,I6yz,I6zz];
loss = check_inertia_condition(I6,loss,penalty);
% link 7
I7 = [I7xx,I7xy,I7xz ; I7xy,I7yy,I7yz ; I7xz,I7yz,I7zz];
loss = check_inertia_condition(I7,loss,penalty);

end