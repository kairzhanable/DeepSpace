using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Auto Exposure during runtime.")]
	public class PostProcessingAutoExposure : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Auto Exposure Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Auto Exposure Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate or deactivate Filtering Parameter")]
		public FsmBool FilteringActive;
		[Tooltip("Use X as Minimum Value and Y as Maximum from 0 to 100 %")]
		public FsmVector2 FilteringMinMax;
		[HasFloatSlider(0f, 100f)]
		public FsmFloat XValue;
		[HasFloatSlider(0f, 100f)]
		public FsmFloat YValue;
		
		[Tooltip("Activate or deactivate Minimum EV")]
		public FsmBool MinimumEvActive;
		[HasFloatSlider(-9f, 9f)]
		[Tooltip("Min Filter")]
		public FsmFloat MinimumEv;

		[Tooltip("Activate or deactivate Maximum EV")]
		public FsmBool MaximumEvActive;
		[HasFloatSlider(-9f, 9f)]
		[Tooltip("Max Filter")]
		public FsmFloat MaximumEv;

		[Tooltip("Activate or deactivate Exposure Compensation")]
		public FsmBool ExposureActive;
		[HasFloatSlider(0f, 1000f)]
		[Tooltip("Exposure compensation")]
		public FsmFloat ExposureCompensation;
		
		[Tooltip("Activate or deactivate Adaptation Type")]
		public FsmBool AdaptationActive;
		[Tooltip("Set Adaption Type")]
		public EyeAdaptation AdaptationType;

		[Tooltip("Activate or deactivate Exposure Compensation")]
		public FsmBool SpeedUpActive;
		[HasFloatSlider(0f, 100f)]
		[Tooltip("Exposure compensation")]
		public FsmFloat SpeedUp;

		[Tooltip("Activate or deactivate Exposure Compensation")]
		public FsmBool SpeedDownActive;
		[HasFloatSlider(0f, 100f)]
		[Tooltip("Exposure compensation")]
		public FsmFloat SpeedDown;

		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private AutoExposure AutoExpLayer;
		private PostProcessProfile UsedProfile; 		
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			FilteringActive = true;
			FilteringMinMax = new Vector2(50,95);
			XValue = new FsmFloat{ UseVariable = true};
			YValue = new FsmFloat{ UseVariable = true};
			MinimumEvActive = true;
			MinimumEv = 0f;
			MaximumEvActive = true;
			MaximumEv = 0f;
			ExposureActive = true;
			ExposureCompensation =1f;
			AdaptationActive = true;
			AdaptationType = 0;
			SpeedUpActive = true;
			SpeedUp = 2f;
			SpeedDownActive = true;
			SpeedDown = 1f;
			everyFrame = false;
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			setAutoExposureParameters();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
				setAutoExposureParameters();
		}

		void setAutoExposureParameters()
		{
							
				UsedProfile = ObjectProfile.Value as PostProcessProfile;				
				UsedProfile.TryGetSettings(out AutoExpLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<AutoExposure>())
				{
					if(AutoExpLayer!=null)
					{
						AutoExpLayer.active = ActiveLayer.Value; //Active Auto Exposure Layer
						AutoExpLayer.enabled.value = ActiveEffect.Value; // Active Auto Exposure Effect
						AutoExpLayer.filtering.overrideState = FilteringActive.Value ;//Activate Focus Distance
						if(FilteringActive.Value)
						{
							if (FilteringMinMax == null) return;
			
							Vector2 newVector2 = FilteringMinMax.Value;
							
							if (!FilteringMinMax.IsNone) newVector2 = FilteringMinMax.Value;
							if (!XValue.IsNone) newVector2.x = XValue.Value;
							if (!YValue.IsNone) newVector2.y = YValue.Value;
							
							FilteringMinMax.Value = newVector2;

							AutoExpLayer.filtering.value = FilteringMinMax.Value ; // set distance value
						}
						AutoExpLayer.minLuminance.overrideState = MinimumEvActive.Value ;//Activate Min Luminance
						if(MinimumEvActive.Value)
						{
							AutoExpLayer.minLuminance.value = MinimumEv.Value ; // set Minimum Luminance value
						}
						AutoExpLayer.maxLuminance.overrideState = MaximumEvActive.Value ;//Activate Max Luminance 
						if(MaximumEvActive.Value)
						{
							AutoExpLayer.maxLuminance.value = MaximumEv.Value ; // set Maximum Luminance value
						}
						AutoExpLayer.keyValue.overrideState = ExposureActive.Value ;//Activate Exposure Compensation 
						if(ExposureActive.Value)
						{
							AutoExpLayer.keyValue.value = ExposureCompensation.Value ; // set Exposure Compensation value
						}
						AutoExpLayer.eyeAdaptation.overrideState = AdaptationActive.Value ;//Activate Adaptation Type 
						if(AdaptationActive.Value)
						{
							AutoExpLayer.eyeAdaptation.value = AdaptationType ; // set Adaptation Type
						}

						AutoExpLayer.speedUp.overrideState = SpeedUpActive.Value ;//Activate Speed Up Parameter 
						if(SpeedUpActive.Value)
						{
							AutoExpLayer.speedUp.value = SpeedUp.Value ; // set Speed Up
						}
						AutoExpLayer.speedDown.overrideState = SpeedDownActive.Value ;//Activate Speed Doawn Parameter 
						if(SpeedDownActive.Value)
						{
							AutoExpLayer.speedDown.value = SpeedDown.Value ; // set Speed Down
					}
				}
			}
		}
	}
}


