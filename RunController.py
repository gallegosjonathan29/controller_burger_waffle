#!/usr/bin/env python  
import rospy
from math import sqrt,cos,sin,tanh,atan2,pi,tanh,cosh
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
import sys, select, termios, tty
import time
import datetime as DT
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64


def miapagado():
    try:
        fh.close()
    except:
        print('El archivo ya estaba cerrado gracias por preguntar')
    print "Me voy apagar!"
    msgf = geometry_msgs.msg.Twist()
    msgf.angular.z =0.0
    msgf.linear.x = 0.0
    turtle_vel1.publish(msgf)
    turtle_vel2.publish(msgf)



def controladorR1(x1,y1,theta1,x1d,y1d,theta1d,v1d,w1d):
    k=0.2
    k3=0.5
    x1_tilde=x1-x1d
    y1_tilde=y1-y1d
    theta1_tilde=theta1-theta1d
    errorx1=-cos(theta1)*x1_tilde-sin(theta1)*y1_tilde
    errory1=sin(theta1)*x1_tilde-cos(theta1)*y1_tilde
    errorTheta1=-theta1_tilde
    v1=v1d*cos(errorTheta1)+k*errorx1
    w1=w1d+k3*errorTheta1+v1d*errory1*(sin(errorTheta1)/errorTheta1)
    return v1,w1,errorx1,errory1,errorTheta1

def trayectoriaR1(t):
    r=0
    l=1.5
    t1=50.0
    if (t>=0.0) & (t<=t1):
        ts1=t-0.0#tiempo para este segmento.
        a0=l/t1#Constante para que termine en tiempo
        q1=a0*ts1#Esta es la funcion que describe la variacion del parametro x
        q1p=a0#primera derivada
        #Posiciones----------------
        x_s1=q1-0.5
        y_s1=r
        xp_s1=q1p#derivada1 posicion x
        yp_s1=0.0#derivada1 posicion y
        theta_s1=0.0#posicion theta
        thetap_s1=0.0#derivada1 posicion theta
        tecla='w'
        x1d=x_s1
        y1d=y_s1
        theta1d=theta_s1
        x1dp=xp_s1
        y1dp=yp_s1
        theta1dp=thetap_s1
        v1d=sqrt(x1dp**2+y1dp**2)
        w1d=thetap_s1
        
    else:
        tecla =' '
        x1d=0.0
        y1d=0.0
        theta1d=0.0
        x1dp=0.0
        y1dp=0.0
        theta1dp=0.0
        v1d=0.0
        w1d=0.0
        #fh.close()
    return x1d, y1d, theta1d, x1dp, y1dp,theta1dp,v1d,w1d,tecla


def trayectoriaR2(t,x1,y1,theta1,v1,w1,x2,y2,theta2):
    x1p=v1*cos(theta1)
    y1p=v1*sin(theta1)
    v1p=0.0
    w1p=0.0
    x1pp=v1p*cos(theta1)-v1*sin(theta1)*w1
    y1pp=v1p*sin(theta1)+v1*cos(theta1)*w1
    #Para el angulo deseado y la distancia deseada.
    w=1.0/5.0
    Am=0.05
    L=0.5+Am*sin(w*t)
    Lp=Am*cos(w*t)*w
    Lpp=-Am*sin(w*t)*(w**2)
    #------------------
    a0=1.5708
    a1=1.9984e-17
    a2=-1.1102e-18  
    a3=0.00012566
    a4=-3.7699e-06
    a5=3.0159e-08;
    fi= a5 * t ** 5 + a4 * t ** 4 + a3 * t ** 3 + a2*t**2 + a1*t+a0
    fip=5 * a5 * t ** 4 + 4 * a4 * t ** 3 + 3 * a3 * t ** 2 + 2 * a2 * t + a1
    fipp=20 * a5 * t **3 + 12 * a4 * t ** 2 + 6 * a3 * t + 2 * a2 
    x2d=x1+L*cos(fi)
    x2dp=x1p+L*(-sin(fi)*fip)+Lp*cos(fi)
    x2dpp=x1pp-L*sin(fi)*fipp-L*cos(fi)*((fip)**2)+2*Lp*(-sin(fi)*fip)+Lpp*cos(fi)
    #Posicion deseada Y.
    y2d=y1+L*sin(fi)
    y2dp=y1p+L*(cos(fi)*fip)+Lp*sin(fi)
    y2dpp=y1pp+L*cos(fi)*fipp-L*sin(fi)*((fip)**2)+2*Lp*(cos(fi)*fip)+Lpp*sin(fi)
    #controles deseados
    #v2d=y2dp*sin(theta2d)+cos(theta2d)*x2dp;
    w2d=(x2dp*y2dpp-x2dpp*y2dp)/((x2dp)**2+(y2dp)**2)
    #%-------------------------------------------------
    lx=-(x1-x2)*cos(theta1)-(y1-y2)*sin(theta1)
    ly=(x1-x2)*sin(theta1)-(y1-y2)*cos(theta1)
    lxd=L*cos(fi)
    lyd=L*sin(fi)
    e2x=lxd-lx
    e2y=lyd-ly
    e2thetaFL=theta2-theta1
    #thetad--------------------------
    d=sqrt(e2x**2+e2y**2)
    A1=10.0
    A2=10.0
    a1=0.05
    a2=0.05
    F1=(tanh(A1*(d-a1))+1.0)/2.0
    F2=(-tanh(A2*(d-a2))+1.0)/2.0
    theta2d=atan2((y2d-y2)*F1+y2dp*F2,(x2d-x2)*F1+x2dp*F2)
    f1=-L*fip*sin(fi)-w1*L*sin(fi)+Lp*cos(fi)+v1
    #este termino podria causar problemas REVISAR.
    f1p=-L*((fip**2)*cos(fi)+fipp*sin(fi))-Lp*fip*sin(fi)-w1*(L*cos(fi)*fip+Lp*sin(fi))-w1p*L*sin(fi)-Lp*sin(fi)*fip+Lpp*cos(fi)+v1p
    f2=L*fip*cos(fi)+w1*L*cos(fi)+Lp*sin(fi)
    #------------------
    return w2d, theta2d, L, fi,e2x,e2y,e2thetaFL, f1, f2, x2d, y2d, f1p, w1p


def YWAlphaex(e2x,e2y,e2thetaFL,w1,s1,xi,f1,k1):
    gamma1=0.05#0.05
    k2=10.0
    A=100.0
    alphaex=-k1*e2x-gamma1*tanh(A*s1)+xi*cos(e2thetaFL)-f1
    y=e2y*w1-alphaex
    w=-k2*y
    return y, w, gamma1, k2,A


def sistemaAuxiliar(e2thetaFL,xi,s1,s2,y,w1,w1p,w2,e2x,e2y,f2,f1p,k1,gamma1,k2,A):
    gamma2=0.01#0.1,1.0
    alphaPunto=(-sin(e2thetaFL)*xi-w1*e2x+f2)*w1+e2y*w1p+k1*(y-k1*e2x-gamma1*tanh(A*s1))+gamma1*(A*((1.0/cosh(A*s1))**2))*(y-gamma1*tanh(A*s1))+xi*sin(e2thetaFL)*(w2-w1)+f1p;
    xip=(1.0/cos(e2thetaFL))*(alphaPunto+s1+k2*y+gamma2*tanh(A*s2));
    return xip

def controladorR2(w2d,xi,theta2d,theta2):
    v2=xi
    k3=5.0
    error2ThetaF=theta2d-theta2
    w2=w2d+k3*error2ThetaF
    return v2, w2, error2ThetaF


if __name__ == '__main__':
    rospy.init_node('controlador1')#nombre
    tfBuffer = tf2_ros.Buffer()#comunicacion
    listener = tf2_ros.TransformListener(tfBuffer)#comunicacion
    turtle_vel1 = rospy.Publisher('/cmd_vel_waffle', geometry_msgs.msg.Twist, queue_size=1)
    turtle_vel2 = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    prueba1=rospy.Publisher('/prueba/p1', Float64, queue_size=1)
    prueba2=rospy.Publisher('/prueba/p2', Float64, queue_size=1)
    prueba3=rospy.Publisher('/prueba/p3', Float64, queue_size=1)
    prueba4=rospy.Publisher('/prueba/p4', Float64, queue_size=1)
    prueba5=rospy.Publisher('/prueba/p5', Float64, queue_size=1)
    control_v1=0.0
    control_w1=0.0
    control_v2=0.0
    control_w2=0.0
    #-------------Constantes---------------
    linear_vel1 = 0.0
    angular_vel1 = 0.0
    linear_vel2 = 0.0
    angular_vel2 = 0.0
    rate = rospy.Rate(100)
    tecla = 'w'#teclado()#'w'
    msg1 = geometry_msgs.msg.Twist()
    msg2 = geometry_msgs.msg.Twist()
    rospy.on_shutdown(miapagado)
    """k1=0.8
    k2=0.8
    k3=0.5
    #--------Para reparar el angulo 1
    m1_s0=0.0
    m1_s1=0.0
    theta1=0.0
    cuenta1=0"""
    #--------Para reparar el angulo 2
    m2_s0=0.0
    m2_s1=0.0
    theta2=0.0
    deri2=0.0
    acumulador2=0.0
    cuenta2=0
    #--------Para el tiempo.------------
    antes=DT.datetime.now()
    ahora=DT.datetime.now()
    taux=0.0
    t=0.0
    cuenta=0
    deltat=0.0
    #Fin de las variables del tiempo---
    #Para abrir archivo-----------
    fh = open('datos.txt', 'a') 
    #Para derivar-----------------
    #s0_px=0.0
    #s1_px=0.0
    #dx=0.0
    #cuenta_px=0
    #s0_py=0.0
    #s1_py=0.0
    #dy=0.0
    #cuenta_py=0
    #s0_ptheta=0.0
    #s1_ptheta=0.0
    #dtheta=0.0
    #cuenta_ptheta=0
    #-Para la integral.
    #int_1_tiempo=np.zeros(3,dtype=float)
    #int_1_datos=np.zeros(3,dtype=float)
    #Para las integrales.
    e2x_ant1=0.0
    v_ant1=0.0
    xip_ant1=0.0
    Integral_e2x=0.0#Condicion inicial 00000000000000000.
    xi=0.0#Condicion inicial 0000000000000000000.
    Integral_v=0.0#Condicion inicial 00000000000000000.
    while not rospy.is_shutdown():
        antes=ahora
        ahora = DT.datetime.now()
        try:
            trans1 = tfBuffer.lookup_transform('world','RigidBody1', rospy.Time())
            trans2 = tfBuffer.lookup_transform('world', 'RigidBody2', rospy.Time())
            if cuenta==0:
                cuenta=1
                print "Me ejecuto una sola vez"
                taux=0.0
                #Condiciones iniciales.
                Integral_e2x=0.0#Condicion inicial 00000000000000000.
                xi=0.0#Condicion inicial 0000000000000000000.
                Integral_v=0.0#Condicion inicial 00000000000000000.
                w2=0.0
                
            else:
                deltat=(ahora-antes).total_seconds()
                taux=taux+deltat
            t=taux
            #Trayectoria deseada.
            #x1d, y1d, theta1d, x1dp, y1dp,theta1dp,v1d,w1d,tecla=trayectoriaR1(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "Espere aun no obtengo las transformadas"
            rate.sleep()
            continue
        
        #primer robot.-----------------------------------
        quaternion1=[trans1.transform.rotation.x,trans1.transform.rotation.y,trans1.transform.rotation.z,trans1.transform.rotation.w]
        euler1 = euler_from_quaternion(quaternion1)
        roll1 = euler1[0]
        pitch1 =euler1[1]
        yaw1 = euler1[2]
        anguloAcompensar1=roll1
        theta1=anguloAcompensar1
        #segundo robot--------------------------------
        quaternion2=[trans2.transform.rotation.x,trans2.transform.rotation.y,trans2.transform.rotation.z,trans2.transform.rotation.w]
        euler2 = euler_from_quaternion(quaternion2)
        roll2 = euler2[0]
        pitch2 =euler2[1]
        yaw2 = euler2[2]
        anguloAcompensar2=roll2
        #theta2=anguloAcompensar2
        #-----Aqui comienza el algoritmo para evitar el salto de la deseada-----------------
        if cuenta2==0:
            cuenta2=1
            print "Me ejecuto una sola vez para otra cosa para robot2"
            m2_s0=0;
            m2_s1=anguloAcompensar2
            acumulador2=0.0#-0.0301#0#-0.0162994
            theta2=acumulador2+anguloAcompensar2
        else:
            m2_s0=m2_s1
            m2_s1=roll2
            #Se obtiene la derivada2
            deri2=(m2_s1-m2_s0)/0.001
            umbral2=1000
            if deri2==0.0:
                acumulador2=acumulador2
            elif ((deri2>0.0) & (deri2<umbral2)):
                acumulador2=acumulador2
            elif ((deri2<0.0) & (deri2>-umbral2)):
                acumulador2=acumulador2
            elif deri2<-umbral2:
                acumulador2=acumulador2+2.0*pi
            elif deri2>umbral2:
                acumulador2=acumulador2-2.0*pi            
            theta2=acumulador2+anguloAcompensar2
        print("angulo normal: ",anguloAcompensar2,"Angulo transformado",theta2)
        #Lo que se produce es el angulo theta2 corregido cuando da saltos
        #---------------------------------------------
        if tecla == 'w':
            #Robot1
            x1=-trans1.transform.translation.x
            y1=trans1.transform.translation.z
            #Robot2
            x2=-trans2.transform.translation.x
            y2=trans2.transform.translation.z
            #Inicio----------------------------------------------------------------------------------------------
            x1d, y1d, theta1d, x1dp, y1dp,theta1dp,v1d,w1d,tecla=trayectoriaR1(t)#Deseada1
            #x1=0.5-0.001
            #y1=1.5-0.001
            #theta1=0.001
            v1,w1,errorx1,errory1,errorTheta1=controladorR1(x1,y1,theta1,x1d,y1d,theta1d,v1d,w1d)#Control1
            #x2=0.0
            #y2=2.5+0.5
            #theta2=-pi/2
            w2d, theta2d, L, fi,e2x,e2y,e2thetaFL, f1, f2, x2d, y2d, f1p, w1p=trayectoriaR2(t,x1,y1,theta1,v1,w1,x2,y2,theta2)#Deseada2
            #print "x1d, y1d, theta1d, x1dp, y1dp,theta1dp,v1d,w1d,tecla"
            #print x1d, y1d, theta1d, x1dp, y1dp,theta1dp,v1d,w1d,tecla
            #print "v1,w1,errorx1,errory1,errorTheta1"
            #print v1,w1,errorx1,errory1,errorTheta1
            #print "w2d, theta2d, L, fi,e2x,e2y,e2thetaFL, f1, f2, x2d, y2d, f1p, w1p"
            #print w2d, theta2d, L, fi,e2x,e2y,e2thetaFL, f1, f2, x2d, y2d, f1p, w1p
            k1=0.001
            s1=k1*Integral_e2x+e2x
            y, w, gamma1, k2,A=YWAlphaex(e2x,e2y,e2thetaFL,w1,s1,xi,f1,k1)
            #print "y, w, gamma1, k2,A"
            #print y, w, gamma1, k2,A
            s2=y-Integral_v
            xip=sistemaAuxiliar(e2thetaFL,xi,s1,s2,y,w1,w1p,w2,e2x,e2y,f2,f1p,k1,gamma1,k2,A)
            #print "xip"
            #print xip
            #-----------------------Control-----------------
            v2, w2, error2ThetaF=controladorR2(w2d,xi,theta2d,theta2)
            #print "v2, w2, error2ThetaF"
            #print v2, w2, error2ThetaF
            v=-s1+w#
            #integrar xip resulta en xi
            #integrar v resulta en Iv
            #integrar ex resulta en Iex
            #Integrales.
            Integral_e2x=0.5*(e2x+e2x_ant1)*deltat+Integral_e2x
            Integral_v=0.5*(v+v_ant1)*deltat+Integral_v
            xi=0.5*(xip+xip_ant1)*deltat+xi
            #Corrimiento de los datos anteriores.
            e2x_ant1=e2x
            v_ant1=v
            xip_ant1=xip
            #Estableciendo limites para los controles.
            if v2>=0.1:
                v2=0.1
            elif v2<=-0.1:
                v2=-0.1
            else:
                v2=v2
            if w2>=0.2:
                w2=0.2
            elif w2<=-0.2:
                w2=-0.2
            else:
                w2=w2
            control_v1=v1
            control_w1=w1
            control_v2=v2
            control_w2=w2
            #print('control 2')"""
            #para las pruebas derivada de X--------------
            """delta=(ahora-antes).total_seconds()
            if cuenta_px==0:
                cuenta_px=1
                s0_px=0;
                s1_px=x
                dx=(s1_px-s0_px)/delta
            else:
                s0_px=s1_px
                s1_px=x
                dx=(s1_px-s0_px)/delta
            if cuenta_py==0:
                cuenta_py=1
                s0_py=0;
                s1_py=y
                dy=(s1_py-s0_py)/delta
            else:
                s0_py=s1_py
                s1_py=y
                dy=(s1_py-s0_py)/delta
            if cuenta_ptheta==0:
                cuenta_ptheta=1
                s0_ptheta=0;
                s1_ptheta=theta
                dtheta=(s1_ptheta-s0_ptheta)/delta
            else:
                s0_ptheta=s1_ptheta
                s1_ptheta=theta
                dtheta=(s1_ptheta-s0_ptheta)/delta
            v_real=math.sqrt(dx**2+dy**2)
            w_real=dtheta"""
            #fin de las pruebas-------------------------
            factor_waffle_v=1.0991986138184
            factor_waffle_w=1.0844017094017
            factor_burger1_v=1.0991986138184
            factor_burger1_w=1.0844017094017
            linear_vel1 =factor_waffle_v*control_v1#control_v#0.05
            angular_vel1=factor_waffle_w*control_w1#control_w#0.1
            linear_vel2 =factor_burger1_v*control_v2#control_v#0.05
            angular_vel2=factor_burger1_w*control_w2#control_w#0.1
            #Escritura de datos en txt, tiempo xtilde y tilde controlv controlw
            cadena=str(t)+' '+str(e2x)+' '+str(e2y)+' '+str(error2ThetaF)+' '+str(v2)+' '+str(w2)+' '+str(x2)+' '+str(y2)+' '+str(theta2)+' '+str(x1)+' '+str(y1)+' '+str(theta1)+' '+str(x2d)+' '+str(y2d)+' '+str(theta2d)+'\n'
            print t
            fh.write(cadena)
            #publicacion para las pruebas
        elif tecla == ' ' :
            angular_vel1 =0.0
            linear_vel1= 0.0
            angular_vel2 =0.0
            linear_vel2= 0.0
            print "Me detengo"
            fh.close()
        else:
            angular_vel1 =angular_vel1
            linear_vel1=linear_vel1
            angular_vel2 =angular_vel2
            linear_vel2=linear_vel2
        #Para publicar las velocidades.
        msg1.angular.z =angular_vel1
        msg1.linear.x = linear_vel1
        msg2.angular.z =angular_vel2
        msg2.linear.x = linear_vel2
        #Para depuracion-----------------------
        pru1 =e2x
        pru2 =e2y
        pru3 =error2ThetaF
        pru4 =v2
        pru5 =w2
        prueba1.publish(pru1)
        prueba2.publish(pru2)
        prueba3.publish(pru3)
        prueba4.publish(pru4)
        prueba5.publish(pru5)
        #--------------------------------------
        turtle_vel1.publish(msg1)
        turtle_vel2.publish(msg2)
        rate.sleep()
