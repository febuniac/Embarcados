	/**

			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if (pstrRecv && pstrRecv->s16BufferSize > 0) {
				printf(pstrRecv->pu8Buffer);	
				for (int i =0; i< pstrRecv->s16BufferSize ;i++)
				{
					if (pstrRecv->pu8Buffer[i] =='$')
					{		
						valor[0]=pstrRecv->pu8Buffer[i+5];
						valor[1]=pstrRecv->pu8Buffer[i+6];
						valor[2]=NULL;
						Temp1= atoi(valor);
						printf("valor: %s \n ",valor);
						printf("temp1 = %d \n", Temp1);
					}
				}
			
				*/
			
				//printf(pstrRecv->pu8Buffer);
				//const char needle[10] = "Temp2";
				//char *ret;
				//   printf("aquuiii \n");
				//
				//   ret = strstr(pstrRecv->pu8Buffer, needle);
				//
				//   printf("The substring is: %s\n", ret);

				//printf(pstrRecv->pu8Buffer);