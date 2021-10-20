using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Grain During Runtime.")]
	public class PostProcessingGrain : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Grain Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Grain Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate Colored parameter")]
		public FsmBool ColoredActive;
		[Tooltip("Is Grain Colored ?")]
		public FsmBool Colored;

		[Tooltip("Activate Intensity parameter")]
		public FsmBool IntensityActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("Grain Intensity")]
		public FsmFloat Intensity;

		[Tooltip("Activate Size parameter")]
		public FsmBool SizeActive;
		[HasFloatSlider(0.3f, 3f)]
		[Tooltip("Grain Size")]
		public FsmFloat Size;

		[Tooltip("Activate Luminance Contribution parameter")]
		public FsmBool LuminanceContributionActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("Luminance Contribution")]
		public FsmFloat LuminanceContribution;


		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion

		#region Private variables
		private Grain GrainLayer;
		private PostProcessProfile UsedProfile; 		
		#endregion

		public override void Reset()
		{
			ActiveLayer= true;
			ActiveEffect = true;
			ObjectProfile = null;
			Intensity = 0f;
			SizeActive = true;
			Size = 1f;
			LuminanceContributionActive = true;
			LuminanceContribution = 0.8f;
			everyFrame = false;
			
		}

		// Code that runs on entering the state.
		public override void OnEnter()
		{
			SetGrain();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetGrain();
		}

		void SetGrain()
		{
							
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out GrainLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<Grain>())
				{
					
					if(GrainLayer!=null)
					{
						GrainLayer.active = ActiveLayer.Value; //Active AO Layer
						GrainLayer.enabled.value = ActiveEffect.Value; // Active AO Effect
						GrainLayer.colored.overrideState=ColoredActive.Value;//Activate Mode
						if(ColoredActive.Value)
						{ 	
							GrainLayer.colored.value = Colored.Value; // AO Mode selection
						}
						GrainLayer.intensity.overrideState=IntensityActive.Value;//Activate Mode
						if(IntensityActive.Value)
						{ 	
							GrainLayer.intensity.value = Intensity.Value; // AO Mode selection
						}
						GrainLayer.size.overrideState = SizeActive.Value;//Activate Mode
						if(SizeActive.Value)
						{ 	
							GrainLayer.size.value = Size.Value; // AO Mode selection
						}
						GrainLayer.lumContrib.overrideState = LuminanceContributionActive.Value;//Activate Mode
						if(LuminanceContributionActive.Value)
						{ 	
							GrainLayer.lumContrib.value = LuminanceContribution.Value; // AO Mode selection
						}
					}
				}
			}
		}
}


