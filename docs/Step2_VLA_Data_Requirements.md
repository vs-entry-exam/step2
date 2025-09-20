# **Key Data Requirements for Vision-Language-Action (VLA) Model Training**

시각-언어-행동(VLA) 모델을 훈련하려면 시각, 언어, 행동 정보를 결합한, 신중하게 선별되고 다양하며 다중 모드의 데이터 세트가 필요합니다. 효과적인 VLA 훈련을 위한 주요 데이터 요구 사항과 전략은 다음과 같습니다.

## Core Data Types Needed

- **Vision Data:** 환경, 사물 및 관련 맥락을 포착하는 고품질 이미지 또는 비디오. 로봇 공학의 경우, 여기에는 다중 시점 RGB 이미지, 깊이 맵, 또는 공간 추론을 위한 3D 포인트 클라우드가 포함되는 경우가 많습니다.  (Arai et al., 2024; Zhen et al., 2024; Li et al., 2025; Huang et al., 2023).
- **Language Data:** 시각적 장면과 의도된 동작에 대응하는 자연어 지시, 설명 또는 대화. 이러한 지시, 설명 또는 대화는 견고한 언어 기반을 지원하기 위해 상세하고 다양해야 합니다.  (Arai et al., 2024; Kim et al., 2024; Zhen et al., 2024; Brohan et al., 2023; Huang et al., 2023).
- **Action Data:** 로봇 동작 또는 궤적의 시퀀스(일반적으로 관절 위치, 속도 또는 엔드 이펙터 자세)를 시각 및 언어 데이터와 연계합니다. 이는 실제 로봇, 시뮬레이션 또는 심지어 인간 시연 영상에서 수집할 수 있습니다.  (Wen et al., 2024; Deng et al., 2025; Kim et al., 2024; Zhang et al., 2024; Ye et al., 2024; Pertsch et al., 2025; Li et al., 2025; Wen et al., 2025; Brohan et al., 2023).

## Data Collection and Efficiency Strategies

- **Large-Scale, Diverse Demonstrations:** VLA models benefit from large and diverse datasets, such as hundreds of thousands to millions of robot demonstrations covering a wide range of tasks, objects, and environments  (Arai et al., 2024; Deng et al., 2025; Kim et al., 2024; Zhang et al., 2024; Pertsch et al., 2025; Brohan et al., 2023; Huang et al., 2023).
- **Synthetic and Simulated Data:** Synthetic datasets (e.g., photorealistic simulations) can supplement or replace costly real-world data, enabling large-scale pretraining and improved generalization  (Deng et al., 2025; Zhen et al., 2024; Fang et al., 2025).
- **Task Decomposition:** Decoupling tasks into sub-stages (e.g., spatial reasoning vs. physical interaction) allows leveraging inexpensive data (like spatial reasoning) to boost performance while minimizing the need for costly physical interaction data  (Zheng et al., 2025).
- **Internet-Scale and Web Data:** Pretraining on internet-scale vision-language datasets, including videos without explicit action labels, can improve generalization and reduce reliance on robot-specific data  (Ye et al., 2024; Brohan et al., 2023; Huang et al., 2023).
- **3D and Multimodal Integration:** Incorporating 3D data (point clouds, 3D coordinates) enhances spatial reasoning and adaptability to real-world scenarios  (Zhen et al., 2024; Li et al., 2025; Huang et al., 2023).

## Training and Curation Considerations

- **Data Quality and Annotation:** 고품질의 주석이 잘 달린 데이터(성공/실패 라벨, 작업 세분화, 언어 정렬)가 매우 중요합니다. 품질이 낮거나 유휴 상태인 데이터를 필터링하면 학습 효율성이 향상됩니다.  (Kim et al., 2024; Pertsch et al., 2025; Huang et al., 2023).
- **Balanced Task Distribution:** 데이터 세트에서 광범위한 기술과 시나리오를 보장하면 보이지 않는 작업과 환경에 대한 강력한 일반화가 지원됩니다.  (Arai et al., 2024; Deng et al., 2025; Ye et al., 2024; Brohan et al., 2023).
- **Efficient Tokenization and Representation:** 동작 데이터의 경우 고급 토큰화 방식(예: 빈도 공간 토큰화)을 사용하면 고빈도의 숙련된 작업에서 학습을 개선할 수 있습니다.  (Pertsch et al., 2025; Wen et al., 2025).

### VLA Training Data Modalities and Sources

| Data Modality         | Typical Sources/Strategies                | Key Considerations                | Citations      |
|----------------------|-------------------------------------------|-----------------------------------|---------------|
| Visual (2D/3D)       | Real/simulated robot cameras, point clouds| Resolution, viewpoint diversity   |  (Arai et al., 2024; Kim et al., 2024; Zhen et al., 2024; Li et al., 2025)|
| Language             | Human-annotated instructions, web data    | Naturalness, task alignment       |  (Arai et al., 2024; Kim et al., 2024; Brohan et al., 2023; Huang et al., 2023)|
| Action               | Robot logs, human demos, synthetic data   | Precision, temporal alignment     |  (Wen et al., 2024; Deng et al., 2025; Kim et al., 2024; Ye et al., 2024)|
| Synthetic/Simulated  | Simulation engines, photorealistic render | Domain randomization, scale       |  (Deng et al., 2025; Zhen et al., 2024; Fang et al., 2025)|
| Web-scale Video      | Internet videos, unsupervised action labels| Skill diversity, transferability  |  (Ye et al., 2024; Brohan et al., 2023; Huang et al., 2023)|

**Figure 1:** Summary of VLA training data types, sources, and considerations.

## Summary

효과적인 VLA 훈련을 위해서는 비전, 언어 및 행동 양식 전반에 걸쳐 방대하고 다양하며 잘 정렬된 데이터 세트가 필요합니다. 합성 데이터, 시뮬레이션 데이터, 웹 스케일 데이터를 활용하고, 신중한 큐레이션 및 고급 토큰화를 병행하면 데이터 효율성과 모델 일반화를 크게 향상시킬 수 있습니다.
 
 
## References
 
Wen, J., Zhu, Y., Li, J., Zhu, M., Tang, Z., Wu, K., Xu, Z., Liu, N., Cheng, R., Shen, C., Peng, Y., Feng, F., & Tang, J. (2024). TinyVLA: Toward Fast, Data-Efficient Vision-Language-Action Models for Robotic Manipulation. *IEEE Robotics and Automation Letters*, 10, 3988-3995. https://doi.org/10.1109/LRA.2025.3544909
 
Zheng, L., Yan, F., Liu, F., Feng, C., Zhong, Y., Huang, Y., & , L. (2025). DataPlatter: Boosting Robotic Manipulation Generalization with Minimal Costly Data. *ArXiv*, abs/2503.19516. https://doi.org/10.48550/arXiv.2503.19516
 
Arai, H., Miwa, K., Sasaki, K., Yamaguchi, Y., Watanabe, K., Aoki, S., & Yamamoto, I. (2024). CoVLA: Comprehensive Vision-Language-Action Dataset for Autonomous Driving. *2025 IEEE/CVF Winter Conference on Applications of Computer Vision (WACV)*, 1933-1943. https://doi.org/10.1109/WACV61041.2025.00195
 
Deng, S., Yan, M., Wei, S., , H., Yang, Y., Chen, J., Zhang, Z., Yang, T., Zhang, X., Cui, H., Zhang, Z., & Wang, H. (2025). GraspVLA: a Grasping Foundation Model Pre-trained on Billion-scale Synthetic Action Data. **. 
 
Kim, M., Pertsch, K., Karamcheti, S., Xiao, T., Balakrishna, A., Nair, S., Rafailov, R., Foster, E., Lam, G., Sanketi, P., Vuong, Q., Kollar, T., Burchfiel, B., Tedrake, R., Sadigh, D., Levine, S., Liang, P., & Finn, C. (2024). OpenVLA: An Open-Source Vision-Language-Action Model. *ArXiv*, abs/2406.09246. https://doi.org/10.48550/arXiv.2406.09246
 
Zhang, J., Wang, K., Wang, S., Li, M., Liu, H., Wei, S., Wang, Z., Zhang, Z., & Wang, H. (2024). Uni-NaVid: A Video-based Vision-Language-Action Model for Unifying Embodied Navigation Tasks. *ArXiv*, abs/2412.06224. https://doi.org/10.48550/arXiv.2412.06224
 
Zhen, H., Qiu, X., Chen, P., Yang, J., Yan, X., Du, Y., Hong, Y., & Gan, C. (2024). 3D-VLA: A 3D Vision-Language-Action Generative World Model. *ArXiv*, abs/2403.09631. https://doi.org/10.48550/arXiv.2403.09631
 
Ye, S., Jang, J., Jeon, B., Joo, S., Yang, J., Peng, B., Mandlekar, A., Tan, R., Chao, Y., Lin, B., Lidén, L., Lee, K., Gao, J., Zettlemoyer, L., Fox, D., & Seo, M. (2024). Latent Action Pretraining from Videos. *ArXiv*, abs/2410.11758. https://doi.org/10.48550/arXiv.2410.11758
 
Pertsch, K., Stachowicz, K., Ichter, B., Driess, D., Nair, S., Vuong, Q., Mees, O., Finn, C., & Levine, S. (2025). FAST: Efficient Action Tokenization for Vision-Language-Action Models. *ArXiv*, abs/2501.09747. https://doi.org/10.48550/arXiv.2501.09747
 
Fang, Y., Yang, Y., Zhu, X., Zheng, K., Bertasius, G., Szafir, D., & Ding, M. (2025). ReBot: Scaling Robot Learning with Real-to-Sim-to-Real Robotic Video Synthesis. *ArXiv*, abs/2503.14526. https://doi.org/10.48550/arXiv.2503.14526
 
Li, C., Wen, J., Peng, Y., Peng, Y., Feng, F., & Zhu, Y. (2025). PointVLA: Injecting the 3D World into Vision-Language-Action Models. *ArXiv*, abs/2503.07511. https://doi.org/10.48550/arXiv.2503.07511
 
Wen, J., Zhu, Y., Li, J., Tang, Z., Shen, C., & Feng, F. (2025). DexVLA: Vision-Language Model with Plug-In Diffusion Expert for General Robot Control. *ArXiv*, abs/2502.05855. https://doi.org/10.48550/arXiv.2502.05855
 
Brohan, A., Brown, N., Carbajal, J., Chebotar, Y., Choromanski, K., Ding, T., Driess, D., Dubey, K., Finn, C., Florence, P., Fu, C., Arenas, M., Gopalakrishnan, K., Han, K., Hausman, K., Herzog, A., Hsu, J., Ichter, B., Irpan, A., Joshi, N., Julian, R., Kalashnikov, D., Kuang, Y., Leal, I., Levine, S., Michalewski, H., Mordatch, I., Pertsch, K., Rao, K., Reymann, K., Ryoo, M., Salazar, G., Sanketi, P., Sermanet, P., Singh, J., Singh, A., Soricut, R., Tran, H., Vanhoucke, V., Vuong, Q., Wahid, A., Welker, S., Wohlhart, P., Xiao, T., Yu, T., & Zitkovich, B. (2023). RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control. *ArXiv*, abs/2307.15818. https://doi.org/10.48550/arXiv.2307.15818
 
Huang, J., Yong, S., , X., Linghu, X., Li, P., Wang, Y., Li, Q., Zhu, S., Jia, B., & Huang, S. (2023). An Embodied Generalist Agent in 3D World. *ArXiv*, abs/2311.12871. https://doi.org/10.48550/arXiv.2311.12871