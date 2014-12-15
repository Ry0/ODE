<pre class="brush: c; first-line: 319; " title="dice.cpp">
int detame()
{
  const dReal *linear_vel;
  const dReal *pos1, *pos2, *pos3, *pos4, *pos5, *pos6;
  double pos[6];
  double max = 0.0;
  int max_num = 0.0;

  linear_vel  = dBodyGetLinearVel(dice.body);//サイコロのそれぞれx,y,z方向の速度を取得

  if(cnt>=300 && linear_vel[0] <= 0.00000001 && linear_vel[1] <= 0.00000001 && linear_vel[2] <= 0.00000001 ){
  //cntはsimloopが何回呼び出されたかのカウンタ、これが300カウントを超えたかつ、x,y,z方向の速度がものすごく小さくなったら
  //サイコロの目の位置を１から６の目に関してゲット
    pos1 = (double *) dBodyGetPosition(one.body);
    pos2 = (double *) dBodyGetPosition(two[0].body);
    pos3 = (double *) dBodyGetPosition(three[0].body);
    pos4 = (double *) dBodyGetPosition(four[0].body);
    pos5 = (double *) dBodyGetPosition(five[0].body);
    pos6 = (double *) dBodyGetPosition(six[0].body);

	//z座標のみ取り出して別の配列に格納
    pos[0] = pos1[2];//1の目のz座標
    pos[1] = pos2[2];//2の目のz座標
    pos[2] = pos3[2];//3の目のz座標
    pos[3] = pos4[2];//4の目のz座標
    pos[4] = pos5[2];//5の目のz座標
    pos[5] = pos6[2];//6の目のz座標

	//これの最大値を求めて一番高い位置にある目を決める
    max = pos[0];
    max_num = 0;
    for(int i = 1; i < 6; i++){
      if(pos[i]>=max){
        max = pos[i];
        max_num = i;
      }
    }
    if(judge == 0){//simloopの中で一回だけ出た目をprintf
      printf("出た目は %d ！！\n", max_num+1);
      judge = 1;
    }
  }else{
    max_num = -1;
  }

  return max_num+1;//一応出た目をリターン
}
</pre>